# SML Universal IR-Scanner für Smartmeter — Protokollanalyse & Programmierunterstützung

**Firmware:** sml_universal_scanner v1.4  
**Hardware:** Wemos D1 Mini o.ä. (ESP8266) + Hichi IR-Lesekopf o.ä. (IR/TTL)  
**Zweck:** Unbekannte SML-Smartmeter mit Infrarot-Augabe analysieren und C++ -Code generieren  
**Datum:** Februar 2026  

---

## 1. Konzept — Dual-Scan (optional)

Der Scanner benötigt zwei Messzyklen (Bezug/Einspeisung) um OBIS-Struktur, Datentypen und
Vorzeichen eines unbekannten Smartmeters vollständig zu charakterisieren:

```
Situation Bezug (A)  →  nur Netzbezug, Keine Einspeisung (Inverter AUS)   [immer erforderlich]
               Smartmeter zeigt positive Momentanleistung
               → Alle Leistungswerte müssen positiv sein

Situation Einspeisung (B)  →  mit Einspeisung, Inverter EIN    [optional — nur bei Einspeise-Inverter]
               Smartmeter zeigt negative Momentanleistung
               → Vorzeichenverhalten der Leistungswerte wird sichtbar
```

Nach Scann-Zyklus Bezug (A) fragt der Scanner ob ein Einspeiseinverter vorhanden ist:

- **Nein (n)** → Scannzyklus Einspeisung (B) wird übersprungen, direkte Analyse und Code-Generierung.
  Alle Leistungswerte werden als unsigned ausgegeben. Im generierten Code
  erscheint ein Hinweis dass das Vorzeichen unbekannt ist.
- **Ja (j)** → Scannzyklus Einspeisung (B) wird durchgeführt. Signed/unsigned und Vorzeichen
  werden durch Vergleich beider Frames eindeutig bestimmt.

Durch den Vergleich beider SML-Frames (Netzbezug/Einspeisung) lassen sich signed/unsigned (Vorzeichen),
Byte-Länge und Scaler für jeden OBIS-Code eindeutig bestimmen — ohne weitere Vorkentnisse
zum Smartmeter.

---

## 2. Hardware & Anschluss

| Komponente | Details |
|---|---|
| Mikrocontroller | Wemos D1 Mini (ESP8266) o.ä. |
| IR-Lesekopf | Hichi IR/TTL-Kopf o.ä (TTL, 9600 Baud) |
| Anschluss | IR-Kopf RX → GPIO14 (D5) |
| Serial Monitor | 115200 Baud, Zeilenende: **Newline (LF)** |

```
Wemos D1 Mini          Hichi IR/TTL-Kopf
─────────────          ─────────────
D5 (GPIO14)  ←────────  Data (TX)
GND          ──────────  GND
3.3V         ──────────  VCC
```

> Kein WLAN, kein MQTT — der Scanner läuft vollständig lokal über Serial Monitor.

---

## 3. Bedienung — Schritt für Schritt

### Schritt 1: Situation Bezug (A)

```
1. Inverter AUS, nur Netzbezug aktiv
2. Scanner auf ESP8266 flashen
3. Serial Monitor öffnen (115200 Baud)
4. Warten bis "Frame A empfangen" erscheint (≤2s)
5. OBIS-Tabelle prüfen
6. Zähleranzeige ablesen → positive Zahl eingeben → ENTER
   Negative Eingabe wird abgewiesen mit Aufforderung zur Korrektur
```

### Schritt 2: Zählerstand bestätigen

```
7. Auto-ermittelter Zählerstand aus OBIS 1-0:1.8.0 wird angezeigt
8. ENTER = übernehmen  |  neuen Wert eingeben = korrigieren
   (Vorbelegung: auto-ermittelter Wert direkt im Eingabepuffer)
```

### Schritt 2.5: Einspeisungs-Abfrage

```
9. Scanner fragt: "Haben Sie einen Solar-Inverter oder andere Einspeisung? (j/n)"

   Eingabe n (Nein):
     → Auswertung Situation Einspeisung (B) wird übersprungen
     → Direkt zu Schritt 4 (Analyse + Code)
     → Hinweis im Code: Vorzeichen unbekannt

   Eingabe j (Ja):
     → Weiter mit Schritt 3
```

Akzeptierte Eingaben: `j` / `J` / `y` / `Y` für Ja, `n` / `N` für Nein.
ENTER allein wird ignoriert — kein versehentliches Auslösen.

### Schritt 3: Situation Einspeisung (B) — mit Einspeisung (nur bei Einspeise-Inverter)

```
10. Inverter einschalten
11. Warten bis Zähler stabil negative Momentanleistung anzeigt
    → Einspeisung muss Eigenverbrauch deutlich übersteigen!
12. Negative Zahl eingeben → ENTER
    Positive Eingabe wird abgewiesen mit Aufforderung zur Korrektur
```

### Schritt 4: Ergebnis

```
13. Analyse erscheint automatisch
14. Generierten Code-Block kopieren → in Production-Sketch einfügen
```

### Timeout-Verhalten

| State | Timeout | Aktion |
|---|---|---|
| Eingabe A (Momentanleistung) | 30 s | `ESP.restart()` |
| Zählerstand-Confirm | 30 s | `ESP.restart()` |
| Inverter-Abfrage (j/n) | 30 s | `ESP.restart()` |
| Eingabe B (Momentanleistung) | 30 s | `ESP.restart()` |
| Warten auf Frame B | 120 s | `ESP.restart()` |

---

## 4. Auto-OBIS-Discovery

Der Scanner sucht ohne Vorwissen nach allen OBIS-Objekten im Frame.

**Suchmuster:** `0x77 0x07` — SML Listen-Tag gefolgt von TL-Feld OBIS-ID

```
Für jedes Match:
  1. 6-Byte OBIS-Kennzahl lesen
  2. Scaler TL (0x52) ab Offset +8 suchen
  3. Value TL direkt danach lesen (0x52..0x59)
  4. Nutzwert-Bytes = (Value TL & 0x0F) - 1
  5. Wert als int64 sign-extended lesen
  6. Wert = raw × 10^Scaler
```

**Duplikat-Erkennung:** Gleiche OBIS-Kennzahl wird nur einmal erfasst.

**Robustheit:** Keine festen Byte-Offsets vorausgesetzt — funktioniert auch wenn
Zähler Felder in anderer Reihenfolge oder mit anderen TL-Typen sendet.

---

## 5. Auto-Phasen-Detect

Nach Frame A werden die gefundenen OBIS-Codes automatisch klassifiziert:

```
L1 erkannt: OBIS 1-0:36.7.0  (0x24 0x07)
L2 erkannt: OBIS 1-0:56.7.0  (0x38 0x07)
L3 erkannt: OBIS 1-0:76.7.0  (0x4C 0x07)

Ausgabe:
  PHASEN: 3-Phasen-Anlage erkannt (L1 + L2 + L3 vorhanden)
  PHASEN: 1-Phasen-Anlage erkannt (nur L1 vorhanden)
  PHASEN: Teilweise erkannt — L1/L2/L3 pruefen
```

---

## 6. OBIS-Tabelle — Klassifizierung

Jedes gefundene OBIS-Objekt wird in der Tabelle mit Typ-Marker versehen:

| Marker | Bedeutung |
|---|---|
| `[POW-SUM] *` | Wirkleistung Summe — Hauptkandidat für `pow` |
| `[POW-L1] *` | Wirkleistung L1 |
| `[POW-L2] *` | Wirkleistung L2 |
| `[POW-L3] *` | Wirkleistung L3 |
| `[ZAEHLER]` | Wirkenergie Bezug (kWh) |
| `[EXPORT-ZAEHLER]` | Wirkenergie Einspeisung (kWh) |
| `[VOLT-Lx]` | Spannung Phase x |
| `[AMP-Lx]` | Strom Phase x |
| `[BLIND-Lx]` | Blindleistung Phase x |
| `[GERAETE-ID]` | Seriennummer / Geräte-ID |
| `[?]` | Unbekannter OBIS-Code |

`*` markiert direkt die Leistungswerte die für den Production-Sketch relevant sind.

---

## 7. Zähler-Validierung & UX

### Eingabe-Validierung

```
Situation Bezug (A):  inputVal < 0  → Abweisung + Re-Prompt
              "FEHLER: Negative Werte in Situation A nicht erlaubt!"
              "> " wird wiederholt, Timeout wird zurückgesetzt

Situation Einspeisung (B):  inputVal >= 0 → Abweisung + Re-Prompt
              "FEHLER: Positive Werte in Situation A?"
              "Einspeisung erhoehen bis Zaehler negativ zeigt"
```

### Zählerstand-Confirm

```
Auto-Ermittlung: OBIS 1-0:1.8.0 aus Frame A
Vorbelegung:     Wert direkt im Eingabepuffer (sichtbar hinter "> ")
Bestätigung:     ENTER übernimmt den Wert unverändert
Korrektur:       Neuen Wert tippen → ENTER ersetzt

Aktualisierung:  In runAnalysis() überschreibt Frame B den Frame-A-Wert
                 (B ist zeitlich aktueller)
```

---

## 8. Export-Counter Highlight

Falls OBIS `1-0:2.8.0` im Frame vorhanden:

```
EXPORT: 1-0:2.8.0 gefunden @ Frame-Pos 117  Wert: 1842.640 kWh
```

Dieser Wert wird in der Code-Generierung als `OBISCOUNTER_OUT` ausgegeben
und kann direkt als Einspeisung-Zähler in den Production-Sketch übernommen werden.

---

## 9. Automatische Code-Generierung

Nach Abschluss des Scans gibt der Scanner einen vollständigen Code-Block aus
der direkt in den Minimal-Sketch (siehe Anhang A) kopiert werden kann.

**Mit Inverter (Dual-Scan):** Vorzeichen bekannt, signed/unsigned korrekt gesetzt.

**Ohne Inverter (nur Situation Bezug (A)):** Zusätzlicher Hinweis im generierten Code:

```cpp
// HINWEIS: Nur Situation Bezug (A) gemessen (kein Inverter).
// Vorzeichen der Leistungswerte unbekannt.
// Alle Werte werden als unsigned ausgegeben.
// Fuer Vorzeichen-Erkennung Scanner mit Inverter wiederholen.
```

**Was der generierte Code enthält:**
- OBIS-Array-Definitionen für alle gefundenen Objekte
- Universelle `readObisValue()` Funktion mit Sign-Extension
- Konkrete Aufrufe mit ermittelten Offsets und Byte-Längen
- TL-Validierung (Scaler TL `0x52` wird geprüft)
- Vorzeichen-Hinweis wenn kein Inverter vorhanden war

Der generierte Block wird als Ganzes in den markierten Bereich
`// === SCANNER OUTPUT: hier einfügen ===` im Minimal-Sketch eingefügt.

---

## 10. TL-Feld Referenz

```
Byte = [Typ 4 Bit][Länge 4 Bit]
Typ 5 = signed int   Typ 6 = unsigned int   Typ 7 = Liste
Länge = GESAMT inkl. TL-Byte → Nutzwert = Länge - 1

0x52 = signed,   2B → 1 Nutzwert-Byte  (Scaler)
0x53 = signed,   3B → 2 Nutzwert-Bytes
0x55 = signed,   5B → 4 Nutzwert-Bytes
0x59 = signed,   9B → 8 Nutzwert-Bytes ← häufigster Typ bei Leistung
0x62 = unsigned, 2B → 1 Nutzwert-Byte  (Unit)
0x65 = unsigned, 5B → 4 Nutzwert-Bytes (Status)
0x77 = Liste,    7B → 6 Listenelemente (OBIS-Objekt-Anfang)
```

**Sign-Extension (kritisch bei negativen Werten):**

```cpp
// Beispiel: 8-Byte Wert mit oberen Bytes 0xFF = negativ
// FF FF FF FF FF FF 5D 65 → int64 = -41627 → *10^-2 = -416.27 W

if (valueLen < 8 && (firstByte & 0x80))
    raw |= ~(((int64_t)1 << (valueLen * 8)) - 1);
```

---

## 11. Bekannte OBIS-Codes

| OBIS | Hex B2/B3 | Beschreibung | Einheit |
|---|---|---|---|
| 1-0:1.8.0 | 01 08 | Wirkenergie Bezug | kWh |
| 1-0:2.8.0 | 02 08 | Wirkenergie Einspeisung | kWh |
| 1-0:15.7.0 | 0F 07 | Wirkleistung Betrag (unsigned) | W |
| 1-0:16.7.0 | 10 07 | Wirkleistung Summe (signed) | W |
| 1-0:36.7.0 | 24 07 | Wirkleistung L1 | W |
| 1-0:56.7.0 | 38 07 | Wirkleistung L2 | W |
| 1-0:76.7.0 | 4C 07 | Wirkleistung L3 | W |
| 1-0:32.7.0 | 20 07 | Spannung L1 | V |
| 1-0:52.7.0 | 34 07 | Spannung L2 | V |
| 1-0:72.7.0 | 48 07 | Spannung L3 | V |
| 1-0:31.7.0 | 1F 07 | Strom L1 | A |
| 1-0:51.7.0 | 33 07 | Strom L2 | A |
| 1-0:71.7.0 | 47 07 | Strom L3 | A |
| 1-0:96.1.0 | 60 01 | Geräte-ID / Seriennummer | — |

---

## 12. Erfahrungswerte Q3AA1054 V10.09

Aus der Entwicklung des Production-Sketches gewonnene Erkenntnisse,
die für andere EasyMeter / ESY-Zähler relevant sein können:

| Eigenschaft | Wert | Anmerkung |
|---|---|---|
| TL-Typ Leistung | `0x59` | 8 Nutzwert-Bytes, int64 signed |
| Scaler Leistung | `0xFE` = -2 | Rohwert in 1/100 W |
| Scaler Energie | `0xFC` = -4 | Rohwert in 1/10000 kWh |
| Sign-Extension | Bytes +15..+20 = `0xFF` bei Einspeisung | Zweierkomplement über alle 8 Bytes |
| OBIS-Suche | 8 Bytes inkl. `0x77` | Sonst False-Positives im Frame |
| Frame-Rate | 1 Hz | Ein Frame pro Sekunde |
| Frame-Größe | ~384 Bytes | Normalframe |

---

*SML Universal IR Scanner v1.4 — Februar 2026*

---

## Anhang A — Beispiel Sketch mit MQTT-Übertragung der Smartmeter-Daten

Dieser Sketch ist direkt compilierbar und lauffähig. Anpassungen sind nur
in den zwei markierten Blöcken nötig: Zugangsdaten und Scanner-Output.

**Benötigte Bibliotheken (Arduino Library Manager):**
- `PubSubClient` (Nick O'Leary)
- `ArduinoJson` (Benoit Blanchon) ≥ v6
- `SoftwareSerial` (in ESP8266-Core enthalten)

---

```cpp
/*
 * SML Smartmeter Reader — Minimal Production Sketch
 * Erstellt mit sml_universal_scanner v1.4
 *
 * Workflow:
 *   1. Scanner flashen → Scan durchführen → Serial Monitor
 *   2. Scanner-Ausgabe in Block "SCHRITT 2" unten einfügen
 *   3. Zugangsdaten in Block "SCHRITT 1" eintragen
 *   4. Flashen → fertig
 *
 * MQTT-Output:
 *   meter/power   → {"pow":0.0,"L1":0.0,"L2":0.0,"L3":0.0}   [W]
 *   meter/counter → {"kwh_in":0.000,"kwh_out":0.000}          [kWh]
 *   meter/status  → "online" / "offline" (LWT)
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
```

---

### Workflow: Scanner → Production

```
1. sml_universal_scanner.ino flashen
2. Serial Monitor (115200 Baud) öffnen
3. Scan durchführen (Situation Bezug (A) + optional B)

4. Scanner gibt aus, z.B.:
     const byte OBIS_POW_SUM[] = {0x77,0x07,0x01,0x00,0x10,0x07,0x00,0xFF};
     ...
     #define POW_SCALER_OFF  13
     #define POW_VALUE_OFF   15
     #define POW_VALUE_LEN    8

5. OBIS-Arrays  → in "SCHRITT 2 / OBIS-Sequenzen" einfügen
   #define-Werte → in "SCHRITT 2 / Lese-Parameter" eintragen

6. WLAN/MQTT    → in "SCHRITT 1" anpassen
7. Production-Sketch flashen → fertig
```

### MQTT-Ausgabe (Beispiel)

```
Topic: meter/power
  {"pow":-416.0,"L1":-696.6,"L2":15.8,"L3":264.8}

Topic: meter/counter
  {"kwh_in":5289.803,"kwh_out":1842.640}

Topic: meter/status
  "online"                                    (retained, LWT: "offline")
```

> **Hinweis Einspeisung:** Negative `pow`-Werte bedeuten Netzeinspeisung.
> Ohne Inverter (nur Situation Bezug (A) gescannt) sind alle Leistungswerte positiv
> und unsigned — der Sketch funktioniert in beiden Fällen identisch.
