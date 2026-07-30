#define WifiSSID "WifiSSID"
#define WifiPassword "Password"
#define MQTTUsername "UserName"
#define MQTTPassword "Password"
#define MQTTHost "Host"
#define SigningKeyHex "000102030405060708090a0b0c0d0e0f"
#define MQTTCACert \
"-----BEGIN CERTIFICATE-----\n" \
"REPLACE_WITH_BROKER_CA_CERT_PEM\n" \
"-----END CERTIFICATE-----\n"
#define OtaUrl "https://OtaHost/garage.bin"
#define OtaUser "OtaUser"
#define OtaPassword "OtaPassword"
#define OTA_CHECK_INTERVAL_MS 3600000
#define OtaRootCA R"EOF(
-----BEGIN CERTIFICATE-----
REPLACE_WITH_OTA_SERVER_ROOT_CA_PEM
-----END CERTIFICATE-----
)EOF"
