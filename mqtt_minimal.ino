/*
 * SML Smartmeter IR-Reader — MQTT Minimal Sketch
 * Erforderlich: Scanner-Output aus sml_universal_scanner_v1.4
 *
 * Workflow:
 *   1. Scanner flashen → Scan durchführen → Serial Monitor-Ausgabe sichern
 *   2. Zugangsdaten in Block "SCHRITT 1" eintragen
 *   3. Scanner-Ausgabe in Block "SCHRITT 2" unten einfügen
 *   4. Flashen → fertig
 *
 * MQTT-Output:
 *   meter/power   → {"pow":0.0,"L1":0.0,"L2":0.0,"L3":0.0}  [W]
 *   meter/counter → {"kwh_in":0.000,"kwh_out":0.000}        [kWh]
 *   meter/status  → "online" / "offline" (LWT)
 *
 * Benötigte Bibliotheken:
 *   PubSubClient (Nick O'Leary)
 *   ArduinoJson (Benoit Blanchon) ≥ v6
 *   SoftwareSerial (in ESP8266-Core enthalten)
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// ============================================================================
// SCHRITT 1: Zugangsdaten anpassen
// ============================================================================
#define SML_RX_PIN      14          // GPIO14 = D5 → Hichi IR-Kopf Data
#define SML_BAUD        9600

const char* WIFI_SSID     = "Netzwerkname";
const char* WIFI_PASS     = "Passwort";
const char* MQTT_SERVER   = "192.168.1.10";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "";     // leer = kein Login erforderlich
const char* MQTT_PASS     = "";
const char* MQTT_CLIENT   = "smartmeter";
const char* TOPIC_POWER   = "meter/power";
const char* TOPIC_COUNTER = "meter/counter";
const char* TOPIC_STATUS  = "meter/status";

#define PUBLISH_POWER_MS    1000UL  // Leistung: jede Sekunde
#define PUBLISH_COUNTER_MS  60000UL // Zähler: jede Minute

// ============================================================================
// SCHRITT 2: Scanner-Output hier einfügen
//            (ersetzt die Beispiel-Einträge unten vollständig)
// ============================================================================

// -- OBIS-Sequenzen (8 Bytes inkl. 0x77 0x07) --------------------------------
// Beispielwerte: durch Scanner-Ausgabe ersetzen!
const byte OBIS_POW_SUM[] = {0x77,0x07,0x01,0x00,0x10,0x07,0x00,0xFF};
const byte OBIS_POW_L1[]  = {0x77,0x07,0x01,0x00,0x24,0x07,0x00,0xFF};
const byte OBIS_POW_L2[]  = {0x77,0x07,0x01,0x00,0x38,0x07,0x00,0xFF};
const byte OBIS_POW_L3[]  = {0x77,0x07,0x01,0x00,0x4C,0x07,0x00,0xFF};
const byte OBIS_KWH_IN[]  = {0x77,0x07,0x01,0x00,0x01,0x08,0x00,0xFF};
const byte OBIS_KWH_OUT[] = {0x77,0x07,0x01,0x00,0x02,0x08,0x00,0xFF}; // optional

// -- Lese-Parameter (aus Scanner-Ausgabe: scalerOff, valueOff, valueLen) -----
// Format: readObisValue(pos, scalerOffset, valueOffset, valueLen)
// scalerOffset = Byte-Abstand OBIS-Start → Scaler-Wert
// valueOffset  = Byte-Abstand OBIS-Start → erster Nutzwert
// valueLen     = Anzahl Nutzwert-Bytes (TL 0x59 → 8, 0x55 → 4, 0x53 → 2)
//
// Beispieloffsets: durch Scanner-Ausgabe ersetzen!
#define POW_SCALER_OFF  13
#define POW_VALUE_OFF   15
#define POW_VALUE_LEN    8

#define CNT_SCALER_OFF  16
#define CNT_VALUE_OFF   18
#define CNT_VALUE_LEN    8

// ============================================================================
// AB HIER NICHTS ÄNDERN
// ============================================================================

#define SML_BUFFER_SIZE  640
#define SML_FRAME_MIN    200

byte         smlBuf[SML_BUFFER_SIZE];
uint16_t     smlPos       = 0;
bool         frameReady   = false;
float        val_pow      = 0.0f;
float        val_L1       = 0.0f;
float        val_L2       = 0.0f;
float        val_L3       = 0.0f;
float        val_kwhIn    = 0.0f;
float        val_kwhOut   = 0.0f;
unsigned long lastPower   = 0;
unsigned long lastCounter = 0;

static bool  smlStarted   = false;
static int   startCnt = 0, endCnt = 0, trailCnt = 0;
static bool  escFound = false, markerFound = false;
const  byte  SML_ESC[]    = {0x1B,0x1B,0x1B,0x1B};
const  byte  SML_BEGIN[]  = {0x01,0x01,0x01,0x01};

SoftwareSerial smlSerial(SML_RX_PIN, -1, false);
WiFiClient     wifiClient;
PubSubClient   mqttClient(wifiClient);

// ── OBIS-Suche ───────────────────────────────────────────────────────────────
int findOBIS(const byte* seq, uint8_t len) {
    if (smlPos < (uint16_t)(len + 26)) return -1;
    for (int i = 0; i <= (int)smlPos - len; i++) {
        if (memcmp(&smlBuf[i], seq, len) == 0) return i;
    }
    return -1;
}

// ── Wert lesen mit Sign-Extension ────────────────────────────────────────────
float readObisValue(int pos, int scalerOff, int valueOff, int valueLen) {
    if (pos < 0) return 0.0f;
    if (pos + valueOff + valueLen >= (int)smlPos) return 0.0f;
    if (smlBuf[pos + scalerOff - 1] != 0x52) return 0.0f;  // Scaler-TL prüfen

    int8_t  scaler = (int8_t)smlBuf[pos + scalerOff];
    int64_t raw    = 0;

    for (int i = 0; i < valueLen; i++)
        raw = (raw << 8) | smlBuf[pos + valueOff + i];

    // Sign-Extension: MSB gesetzt → negative Zahl (Zweierkomplement)
    if (valueLen < 8 && (smlBuf[pos + valueOff] & 0x80))
        raw |= ~(((int64_t)1 << (valueLen * 8)) - 1);

    return (float)raw * powf(10.0f, (float)scaler);
}

// ── SML Frame Parser (Byte für Byte) ─────────────────────────────────────────
void parseSML(byte b) {
    if (!smlStarted) {
        // Start-Sequenz: 1B 1B 1B 1B 01 01 01 01
        const byte startSeq[] = {0x1B,0x1B,0x1B,0x1B,0x01,0x01,0x01,0x01};
        if (b == startSeq[startCnt]) {
            if (++startCnt == 8) {
                smlStarted = true;
                smlPos = startCnt = endCnt = trailCnt = 0;
                escFound = markerFound = false;
            }
        } else {
            startCnt = (b == 0x1B) ? 1 : 0;
        }
        return;
    }

    if (smlPos < SML_BUFFER_SIZE) smlBuf[smlPos++] = b;

    // Ende-Sequenz: 1B 1B 1B 1B 1A xx xx xx
    if (!escFound) {
        if (b == 0x1B) { if (++endCnt >= 4) escFound = true; }
        else endCnt = 0;
    } else if (!markerFound) {
        if (b == 0x1A) markerFound = true;
        else { escFound = false; endCnt = 0; }
    } else {
        if (++trailCnt >= 3) {
            if (smlPos >= SML_FRAME_MIN) frameReady = true;
            smlStarted = false;
            smlPos = endCnt = trailCnt = 0;
            escFound = markerFound = false;
        }
    }

    // Überlauf-Schutz
    if (smlPos >= SML_BUFFER_SIZE - 4) {
        smlStarted = false; smlPos = 0;
    }
}

// ── Decoder: OBIS-Werte aus Frame lesen ──────────────────────────────────────
void decodeSML() {
    int pos;

    pos = findOBIS(OBIS_POW_SUM, 8);
    val_pow    = readObisValue(pos, POW_SCALER_OFF, POW_VALUE_OFF, POW_VALUE_LEN);

    pos = findOBIS(OBIS_POW_L1, 8);
    val_L1     = readObisValue(pos, POW_SCALER_OFF, POW_VALUE_OFF, POW_VALUE_LEN);

    pos = findOBIS(OBIS_POW_L2, 8);
    val_L2     = readObisValue(pos, POW_SCALER_OFF, POW_VALUE_OFF, POW_VALUE_LEN);

    pos = findOBIS(OBIS_POW_L3, 8);
    val_L3     = readObisValue(pos, POW_SCALER_OFF, POW_VALUE_OFF, POW_VALUE_LEN);

    pos = findOBIS(OBIS_KWH_IN, 8);
    val_kwhIn  = readObisValue(pos, CNT_SCALER_OFF, CNT_VALUE_OFF, CNT_VALUE_LEN);

    pos = findOBIS(OBIS_KWH_OUT, 8);
    val_kwhOut = readObisValue(pos, CNT_SCALER_OFF, CNT_VALUE_OFF, CNT_VALUE_LEN);
}

// ── MQTT Publish ─────────────────────────────────────────────────────────────
void publishPower() {
    StaticJsonDocument<128> doc;
    doc["pow"] = serialized(String(val_pow, 1));
    doc["L1"]  = serialized(String(val_L1,  1));
    doc["L2"]  = serialized(String(val_L2,  1));
    doc["L3"]  = serialized(String(val_L3,  1));
    char buf[128];
    serializeJson(doc, buf);
    mqttClient.publish(TOPIC_POWER, buf, true);
}

void publishCounter() {
    StaticJsonDocument<80> doc;
    doc["kwh_in"]  = serialized(String(val_kwhIn,  3));
    doc["kwh_out"] = serialized(String(val_kwhOut, 3));
    char buf[80];
    serializeJson(doc, buf);
    mqttClient.publish(TOPIC_COUNTER, buf, true);
}

// ── WiFi + MQTT Verbindung ────────────────────────────────────────────────────
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(300);
}

void connectMQTT() {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setKeepAlive(30);
    while (!mqttClient.connected()) {
        bool ok = strlen(MQTT_USER) > 0
            ? mqttClient.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS,
                                 TOPIC_STATUS, 0, true, "offline")
            : mqttClient.connect(MQTT_CLIENT,
                                 TOPIC_STATUS, 0, true, "offline");
        if (!ok) delay(2000);
    }
    mqttClient.publish(TOPIC_STATUS, "online", true);
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────
void setup() {
    smlSerial.begin(SML_BAUD);
    connectWiFi();
    connectMQTT();
}

void loop() {
    ESP.wdtFeed();

    if (!mqttClient.connected()) connectMQTT();
    mqttClient.loop();

    while (smlSerial.available())
        parseSML((byte)smlSerial.read());

    if (frameReady) {
        frameReady = false;
        decodeSML();

        unsigned long now = millis();
        if (now - lastPower >= PUBLISH_POWER_MS) {
            publishPower();
            lastPower = now;
        }
        if (now - lastCounter >= PUBLISH_COUNTER_MS) {
            publishCounter();
            lastCounter = now;
        }
    }

}
