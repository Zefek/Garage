# Testy

Ruční testy proti živému zařízení. Nejsou součástí CI – potřebují běžící desku,
broker a vyplněný `secret.h`.

## garage_open_test.py

Zastupuje OdectyStat v secure-open handshaku a ověřuje, že firmware povel
správně přijme i odmítne. Podpisový klíč, credentials, CA i názvy topiců si čte
z `../src/ESP32/config.h` a `secret.h`, takže se nic nezadává ručně.

```
pip install paho-mqtt

python garage_open_test.py            # platný podpis     -> OPENED
python garage_open_test.py --badsig   # prohozený bit     -> BADSIG
python garage_open_test.py --expire   # odpověď až po TTL -> EXPIRED
python garage_open_test.py --replay   # stejný podpis 2x  -> podruhé EXPIRED
```

Návratový kód je 0, když status odpovídá očekávání. Jiný sketch se dá zvolit
přes `--sketch-dir`.

**Posílá skutečný povel k otevření vrat.** Spouštět jen proti sketchi, který má
v `config.h` testovací topicy – proti produkčním názvům by povel dorazil do
garáže. Zkontroluj si to před prvním během.

## Simulace stavového automatu bez hardwaru

Polohu vrat lze odladit dvěma propojkami na GND:

- **`DOORFLASH` na GND** = maják bliká; délka přidržení = délka jízdy
- **`DOORSWITCH` na GND** = vrata nejsou zavřená; odpojení = reed hlásí zavřeno

Průběh se sleduje na retained topicu `GARAGE_STATE` ve tvaru
`<Closed|Open>;<Stop|Opening|Closing|Move>;<poloha 0-100, -1 = neznámá>`.

Ověřit se dá zejména:

- střídání směru při každém rozjezdu (`Opening` -> `Closing` -> `Opening`)
- předblik – rozjezd z `Closed` nezačne počítat, dokud nepustí reed
- detekce reverzace – zavírání delší než ujetá vzdálenost skončí na 100 %
  a nastaví bit `0x02` v `sensorErr`

Naměřené `lastTravelMs` a `lastLeadMs` chodí v `GARAGE_DIAG` a slouží
ke kalibraci `T_FULL_MS` a `T_LEAD_DEFAULT_MS`.
