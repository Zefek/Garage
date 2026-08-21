"""Testovaci podepisovatel pro secure-open handshake garaze.

Zastupuje OdectyStat a overuje, ze firmware handshake spravne prijme i odmitne.
Konfiguraci, credentials i podpisovy klic cte primo z config.h / secret.h
prislusneho sketche, nic se nezadava rucne a nic se nevypisuje na obrazovku.

Pouziti:
    python garage_open_test.py               # platny podpis     -> OPENED
    python garage_open_test.py --badsig      # prohozeny bit     -> BADSIG
    python garage_open_test.py --expire      # odpoved az po TTL -> EXPIRED
    python garage_open_test.py --replay      # stejny podpis 2x  -> podruhe EXPIRED

Vyzaduje paho-mqtt (pip install paho-mqtt).

POZOR: posila povel k otevreni vrat. Spoustet jen proti sketchi, ktery ma
v config.h testovaci topicy, jinak povel dorazi na produkcni garaz.
"""

import argparse
import hashlib
import hmac
import os
import re
import secrets
import ssl
import struct
import sys
import tempfile
import threading
import time

import paho.mqtt.client as mqtt

DEFAULT_SKETCH_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "ESP32")
)

STATUS = {1: "OPENED", 2: "EXPIRED", 3: "BADSIG"}


def parse_defines(path):
    """Vytahne #define NAME hodnota.

    Zvlada obe formy, ktere se v secret.h vyskytuji:
      - viceradkove retezcove konkatenace s backslash pokracovanim
      - surove retezce R"DELIM( ... )DELIM" pres vic radku
    """
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    out = {}

    raw_re = re.compile(r'#define\s+(\w+)\s+R"([^("]*)\((.*?)\)\2"', re.S)
    for m in raw_re.finditer(text):
        out[m.group(1)] = m.group(3).strip() + "\n"
    text = raw_re.sub("", text)

    text = re.sub(r"\\\r?\n", " ", text)
    for line in text.splitlines():
        m = re.match(r'\s*#define\s+(\w+)\s+(.*)', line)
        if not m:
            continue
        name, rest = m.group(1), m.group(2).strip()
        parts = re.findall(r'"((?:[^"\\]|\\.)*)"', rest)
        if parts:
            out[name] = "".join(parts).replace("\\n", "\n").replace('\\"', '"')
        elif rest:
            out[name] = rest
    return out


def sign(key, correlation_id, nonce):
    msg = struct.pack("<I", correlation_id) + nonce + b"open"
    return hmac.new(key, msg, hashlib.sha256).digest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--badsig", action="store_true")
    ap.add_argument("--expire", action="store_true")
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--sketch-dir", default=DEFAULT_SKETCH_DIR)
    ap.add_argument("--timeout", type=float, default=40.0)
    args = ap.parse_args()
    args.replayed = False

    cfg = parse_defines(os.path.join(args.sketch_dir, "config.h"))
    sec = parse_defines(os.path.join(args.sketch_dir, "secret.h"))

    key = bytes.fromhex(sec["SigningKeyHex"])
    nonce_len = int(cfg["GARAGE_NONCE_LEN"])
    ttl_ms = int(cfg["GARAGE_OPEN_TTL"].rstrip("UL"))

    t_req = cfg["GARAGE_OPEN_REQUEST"]
    t_chal = cfg["GARAGE_OPEN_CHALLENGE"]
    t_resp = cfg["GARAGE_OPEN_RESPONSE"]
    t_res = cfg["GARAGE_OPEN_RESULT"]

    print(f"sketch   {args.sketch_dir}")
    print(f"broker   {sec['MQTTHost']}:8883")
    print(f"topicy   {t_req} / {t_chal} / {t_resp} / {t_res}")
    print(f"klic     {len(key)} B, nonce {nonce_len} B, TTL {ttl_ms} ms")

    ca = tempfile.NamedTemporaryFile("w", suffix=".pem", delete=False, encoding="utf-8")
    ca.write(sec["MQTTCACert"])
    ca.close()

    done = threading.Event()
    state = {"r": secrets.randbelow(2**32), "sig": None, "nonce": None, "status": None}

    def on_connect(client, userdata, flags, rc, properties=None):
        client.subscribe([(t_chal, 1), (t_res, 1)])
        payload = struct.pack("<I", state["r"])
        client.publish(t_req, payload, qos=1)
        print(f"\n-> request      R={state['r']}  ({payload.hex(' ')})")

    def on_message(client, userdata, msg):
        if msg.topic == t_chal:
            if len(msg.payload) != 4 + nonce_len:
                print(f"!! challenge ma {len(msg.payload)} B, cekano {4 + nonce_len}")
                return
            r, = struct.unpack("<I", msg.payload[:4])
            nonce = msg.payload[4:]
            print(f"<- challenge    R={r}  nonce={nonce.hex(' ')}")
            if r != state["r"]:
                print(f"!! korelacni id nesedi (cekano {state['r']})")
                return
            state["nonce"] = nonce

            sig = bytearray(sign(key, r, nonce))
            if args.badsig:
                sig[0] ^= 0x01
                print("   (--badsig: prohozen bit 0 podpisu)")
            if args.expire:
                wait = ttl_ms / 1000.0 + 2
                print(f"   (--expire: cekam {wait:.0f} s, aby vyprsel slot)")
                time.sleep(wait)

            state["sig"] = bytes(sig)
            out = struct.pack("<I", r) + state["sig"]
            client.publish(t_resp, out, qos=1)
            print(f"-> response     {len(out)} B  sig={state['sig'].hex(' ')}")

        elif msg.topic == t_res:
            if len(msg.payload) != 5:
                print(f"!! result ma {len(msg.payload)} B, cekano 5")
                return
            r, status = struct.unpack("<IB", msg.payload)
            state["status"] = status
            print(f"<- result       R={r}  status={status} ({STATUS.get(status, '?')})")

            if args.replay and not args.replayed and status == 1:
                print("\n   (--replay: posilam tentyz podpis znovu, slot uz ma byt neplatny)")
                out = struct.pack("<I", r) + state["sig"]
                client.publish(t_resp, out, qos=1)
                args.replayed = True
                return
            done.set()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="GarageOpenTest")
    client.username_pw_set(sec["MQTTUsername"], sec["MQTTPassword"])
    client.tls_set(ca_certs=ca.name, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS_CLIENT)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(sec["MQTTHost"], 8883, 60)
    client.loop_start()

    ok = done.wait(args.timeout)
    client.loop_stop()
    client.disconnect()
    os.unlink(ca.name)

    if not ok:
        print(f"\nTIMEOUT po {args.timeout:.0f} s bez vysledku")
        return 1

    expected = 1
    if args.badsig:
        expected = 3
    elif args.expire or args.replayed:
        expected = 2

    got = state["status"]
    print(f"\n{'OK' if got == expected else 'NESEDI'}: status {got} ({STATUS.get(got, '?')}), cekano {expected} ({STATUS.get(expected, '?')})")
    return 0 if got == expected else 1


if __name__ == "__main__":
    sys.exit(main())
