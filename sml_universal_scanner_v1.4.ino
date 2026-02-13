/*
 * ============================================================================
 * Universeller SML-Scanner v1.4
 * ============================================================================
 * Zweck : Unbekannte Smartmeter analysieren und OBIS-Struktur ableiten.
 *         Ergebnis: fertiger Code-Block für ESP8266 Smart Meter Reader.
 *
 * Hardware: Wemos D1 Mini (ESP8266) + Hichi IR-Lesekopf
 *           IR-Kopf RX → GPIO14 (D5), 9600 Baud 8N1
 *           Kein WLAN, kein MQTT — nur Serial Monitor (115200 Baud)
 *
 * Bedienung (Serial Monitor, 115200 Baud, Zeilenende: Newline):
 *   Nach Boot:  Scanner wartet auf ersten vollständigen SML-Frame
 *   Dann:       Alle gefundenen OBIS-Objekte werden angezeigt
 *
 *   Schritt 1 - Situation A (z.B. nur Bezug, kein Inverter):
 *     Zähleranzeige ablesen → Wert im Serial Monitor eingeben → ENTER
 *     z.B.:  335
 *
 *   Schritt 2 - Situation B (z.B. mit Einspeisung):
 *     Inverter einschalten → warten bis Zähler stabilen Wert zeigt
 *     Zähleranzeige ablesen → Wert eingeben → ENTER
 *     z.B.:  -420
 *
 *   Ausgabe:  Analyse + fertiger Code-Block für den Production-Sketch
 *
 * SML Protokoll:
 *   Frame Start  : 1B 1B 1B 1B 01 01 01 01
 *   Frame Ende   : 1B 1B 1B 1B 1A [fill] [CRC-hi] [CRC-lo]
 *   OBIS-Objekt  : 77 07 [6-Byte-ID] [Status] [Zeit] [Unit] [Scaler] [Value]
 *
 * TL-Feld Kodierung:
 *   Bits 7-4 = Typ: 5=signed int, 6=unsigned int, 7=Liste
 *   Bits 3-0 = Gesamt-Länge inkl. TL-Byte
 *   Nutzwert-Bytes = Länge - 1
 *
 *   0x52 = signed,   2B gesamt → 1 Nutzwert-Byte  (Scaler)
 *   0x53 = signed,   3B gesamt → 2 Nutzwert-Bytes
 *   0x54 = signed,   4B gesamt → 3 Nutzwert-Bytes
 *   0x55 = signed,   5B gesamt → 4 Nutzwert-Bytes
 *   0x59 = signed,   9B gesamt → 8 Nutzwert-Bytes
 *   0x62 = unsigned, 2B gesamt → 1 Nutzwert-Byte  (Unit)
 *   0x65 = unsigned, 5B gesamt → 4 Nutzwert-Bytes (Status)
 *
 * ============================================================================
 */

#define SS_MAX_RX_BUFF 256
#include <SoftwareSerial.h>

// ============================================================================
// KONFIGURATION
// ============================================================================
#define SML_RX_PIN      14    // D5 - IR-Lesekopf
#define SML_TX_PIN      -1    // nicht verwendet
#define SML_BUFFER_SIZE 600
#define SML_MIN_FRAME   200
#define MAX_OBIS        20    // max. OBIS-Objekte pro Frame

// ============================================================================
// SCANNER-ZUSTÄNDE
// ============================================================================
enum ScannerState {
    STATE_WAIT_FRAME_A,      // Warte auf Frame für Situation A
    STATE_INPUT_A,           // Warte auf Momentanleistung A (nur positiv)
    STATE_CONFIRM_COUNTER,   // Zählerstand bestätigen oder korrigieren
    STATE_ASK_INVERTER,      // Abfrage: Inverter vorhanden? (j/n)
    STATE_WAIT_FRAME_B,      // Warte auf Frame für Situation B
    STATE_INPUT_B,           // Warte auf Momentanleistung B (nur negativ)
    STATE_ANALYSE,           // Analyse und Code-Generierung
    STATE_DONE               // Fertig
};

// ============================================================================
// OBIS-OBJEKT STRUKTUR
// ============================================================================
struct ObisObject {
    byte     id[6];         // 6-Byte OBIS-Kennzahl
    int      framePos;      // Position im Frame (Byte 0x77)
    uint8_t  valueTL;       // TL-Byte des Wertefeldes (0x52..0x59)
    uint8_t  valueLen;      // Nutzwert-Bytes (valueTL & 0x0F) - 1
    int8_t   scaler;        // Scaler-Wert (int8)
    int64_t  rawA;          // Rohwert Situation A
    int64_t  rawB;          // Rohwert Situation B
    float    calcA;         // Berechneter Wert A
    float    calcB;         // Berechneter Wert B
    bool     isSigned;      // Typ 5 = signed
    bool     foundInB;      // Auch in Frame B gefunden
};

// ============================================================================
// GLOBALE VARIABLEN
// ============================================================================
SoftwareSerial smlSerial(SML_RX_PIN, SML_TX_PIN);

byte     smlBuffer[SML_BUFFER_SIZE];
uint16_t bufferPos     = 0;
bool     smlStartFound = false;
bool     frameReady    = false;

// SML Parser State
static int  startMatchCount = 0;
static int  endMatchCount   = 0;
static int  endTrailCount   = 0;
static bool endEscFound     = false;
static bool endMarkerFound  = false;

const byte SML_START[] = {0x1B, 0x1B, 0x1B, 0x1B, 0x01, 0x01, 0x01, 0x01};

// Scanner
ScannerState  scanState    = STATE_WAIT_FRAME_A;
ObisObject    obisA[MAX_OBIS];
ObisObject    obisB[MAX_OBIS];
int           obisCountA   = 0;
int           obisCountB   = 0;
float         meterValueA  = 0.0f;   // Momentanleistung Situation A [W]
float         meterValueB  = 0.0f;   // Momentanleistung Situation B [W]
float         counterValue = 0.0f;   // Zählerstand aus Frame A/B [kWh]
char          serialInputBuf[20];
int           serialInputPos = 0;

// Timeout
unsigned long stateEnterTime  = 0;
#define TIMEOUT_INPUT_MS   30000UL   // 30s für Eingabe-States
#define TIMEOUT_FRAME_MS  120000UL   // 120s für Warten auf Frame B

bool hasInverter = false;  // false = Situation B wird übersprungen

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void     parseSML(byte b);
int      discoverOBIS(ObisObject* list, int maxCount);
int64_t  readInt64(int pos, int len);
float    calcValue(int64_t raw, int8_t scaler);
void     printFrameHex();
void     printObisTable(ObisObject* list, int count, const char* label);
const char* classifyObis(byte* id);
void     runAnalysis();
void     generateCode();
void     printSeparator();
bool     readSerialLine(float &result);
String   obisName(byte* id);
String   unitName(uint8_t unitByte);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED aus

    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println(F("\n"));
    Serial.println(F("============================================================"));
    Serial.println(F("  Universeller SML-Scanner v1.0"));
    Serial.println(F("  ESP8266 Smart Meter Reader - Diagnose & Code-Generator"));
    Serial.println(F("============================================================"));
    Serial.println(F("Hardware: Hichi IR-Lesekopf an GPIO14 (D5), 9600 Baud"));
    Serial.println(F("Serial Monitor: 115200 Baud | Zeilenende: Newline (LF)"));
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("\nSchritt 1: Inverter AUS, nur Netzbezug aktiv."));
    Serial.println(F("          Zaehler muss POSITIVE Momentanleistung anzeigen."));
    Serial.println(F("          Warte auf ersten vollstaendigen SML-Frame..."));
    Serial.println();

    smlSerial.begin(9600);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    ESP.wdtFeed();

    // Timeout-Überwachung
    unsigned long now = millis();
    if (scanState == STATE_INPUT_A || scanState == STATE_CONFIRM_COUNTER ||
        scanState == STATE_ASK_INVERTER || scanState == STATE_INPUT_B) {
        if (now - stateEnterTime > TIMEOUT_INPUT_MS) {
            Serial.println(F("\nTimeout Eingabe (30s) — ESP wird neu gestartet..."));
            delay(500);
            ESP.restart();
        }
    }
    if (scanState == STATE_WAIT_FRAME_B) {
        if (now - stateEnterTime > TIMEOUT_FRAME_MS) {
            Serial.println(F("\nTimeout Frame B (120s) — Inverter aktiv? ESP neu starten..."));
            delay(500);
            ESP.restart();
        }
    }

    // SML-Bytes lesen
    while (smlSerial.available()) {
        parseSML(smlSerial.read());
    }

    // Frame verfügbar?
    if (frameReady) {
        frameReady = false;

        if (scanState == STATE_WAIT_FRAME_A) {
            Serial.printf("\nFrame A empfangen: %d Bytes\n", bufferPos);
            obisCountA = discoverOBIS(obisA, MAX_OBIS);

            if (obisCountA == 0) {
                Serial.println(F("FEHLER: Keine OBIS-Objekte gefunden! Frame verwerfen."));
                return;
            }

            printObisTable(obisA, obisCountA, "SITUATION A");
            Serial.println(F("------------------------------------------------------------"));
            Serial.println(F("SITUATION A (nur Bezug, kein Inverter):"));
            Serial.println(F("  -> Nur POSITIVE Momentanleistung erlaubt!"));
            Serial.println(F("  -> Zähleranzeige ablesen und eingeben (z.B. 335)"));
            Serial.print(F("> "));
            scanState = STATE_INPUT_A;
            stateEnterTime = millis();
        }
        else if (scanState == STATE_WAIT_FRAME_B) {
            Serial.printf("\nFrame B empfangen: %d Bytes\n", bufferPos);
            obisCountB = discoverOBIS(obisB, MAX_OBIS);

            if (obisCountB == 0) {
                Serial.println(F("FEHLER: Keine OBIS-Objekte gefunden! Frame verwerfen."));
                return;
            }

            printObisTable(obisB, obisCountB, "SITUATION B");
            Serial.println(F("------------------------------------------------------------"));
            Serial.println(F("SITUATION B (mit Einspeisung):"));
            Serial.println(F("  -> Zähleranzeige ablesen und eingeben (z.B. -420)"));
            Serial.print(F("> "));
            scanState = STATE_INPUT_B;
            stateEnterTime = millis();
        }
    }

    // Serial-Eingabe verarbeiten
    float inputVal;
    if (readSerialLine(inputVal)) {

        if (scanState == STATE_INPUT_A) {
            // Nur positive Werte erlaubt
            if (inputVal < 0.0f) {
                Serial.println(F("FEHLER: Negative Werte in Situation A nicht erlaubt!"));
                Serial.println(F("  -> Inverter ausschalten, warten bis Zaehler positiv,"));
                Serial.println(F("     dann erneut eingeben."));
                Serial.print(F("> "));
                stateEnterTime = millis();  // Timeout zurücksetzen
            } else {
                meterValueA = inputVal;
                Serial.printf("OK: Momentanleistung A gespeichert: +%.1f W\n", meterValueA);

                // Zählerstand aus Frame A ermitteln (OBIS 1-0:1.8.0 suchen)
                counterValue = 0.0f;
                for (int i = 0; i < obisCountA; i++) {
                    if (obisA[i].id[2] == 0x01 && obisA[i].id[3] == 0x08) {
                        counterValue = obisA[i].calcA;
                        break;
                    }
                }

                Serial.println(F("------------------------------------------------------------"));
                Serial.println(F("Zaehlerstand aus Frame A (OBIS 1-0:1.8.0):"));
                if (counterValue > 0.0f) {
                    Serial.printf("  Automatisch ermittelt: %.3f kWh\n", counterValue);
                } else {
                    Serial.println(F("  Nicht gefunden (OBIS 1-0:1.8.0 nicht im Frame)."));
                    Serial.println(F("  Bitte manuell eingeben."));
                    counterValue = 0.0f;
                }
                Serial.println(F("Bestaetigen (ENTER) oder korrigieren (neuen Wert eingeben):"));
                Serial.printf("> ");
                // Eingabepuffer mit aktuellem Wert vorbelegen
                snprintf(serialInputBuf, sizeof(serialInputBuf), "%.3f", counterValue);
                serialInputPos = strlen(serialInputBuf);
                // Vorschlagswert im Terminal anzeigen (Nutzer sieht was er bestätigt)
                Serial.print(serialInputBuf);
                scanState = STATE_CONFIRM_COUNTER;
            }
        }
        else if (scanState == STATE_CONFIRM_COUNTER) {
            // Bestätigung oder Korrektur des Zählerstandes
            if (inputVal >= 0.0f) {
                counterValue = inputVal;
                Serial.printf("OK: Zaehlerstand: %.3f kWh\n", counterValue);
            } else {
                Serial.println(F("HINWEIS: Zaehlerstand kann nicht negativ sein, Wert beibehalten."));
            }
            Serial.printf("OK: Zaehlerstand: %.3f kWh\n", counterValue);
            Serial.println(F("------------------------------------------------------------"));
            Serial.println(F("Haben Sie einen Solar-Inverter oder andere Einspeisung?"));
            Serial.println(F("  j = Ja  → Situation B (Einspeisung) wird gemessen"));
            Serial.println(F("  n = Nein → Situation B wird uebersprungen"));
            Serial.print(F("> "));
            stateEnterTime = millis();
            scanState = STATE_ASK_INVERTER;
        }
        else if (scanState == STATE_INPUT_B) {
            // Nur negative Werte sinnvoll (Einspeisung überwiegt)
            if (inputVal >= 0.0f) {
                Serial.println(F("FEHLER: Positive Werte in Situation B?"));
                Serial.println(F("  -> Einspeisung erhoehen bis Zaehler negativ zeigt,"));
                Serial.println(F("     dann erneut eingeben."));
                Serial.print(F("> "));
                stateEnterTime = millis();  // Timeout zurücksetzen
            } else {
                meterValueB = inputVal;
                Serial.printf("OK: Momentanleistung B gespeichert: %.1f W\n", meterValueB);
                scanState = STATE_ANALYSE;
                runAnalysis();
            }
        }
    }

    // STATE_ASK_INVERTER: j/n direkt aus Serial lesen (kein Float nötig)
    if (scanState == STATE_ASK_INVERTER) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == 'j' || c == 'J' || c == 'y' || c == 'Y') {
                Serial.println(c);
                hasInverter = true;
                Serial.println(F("OK: Situation B wird gemessen."));
                Serial.println(F("------------------------------------------------------------"));
                Serial.println(F("Inverter EINSCHALTEN."));
                Serial.println(F("Einspeisung muss so hoch sein, dass die Momentanleistung"));
                Serial.println(F("auf dem Zaehler NEGATIV angezeigt wird!"));
                Serial.println(F("Warte auf naechsten SML-Frame (Timeout 120s)..."));
                Serial.println();
                stateEnterTime = millis();
                scanState = STATE_WAIT_FRAME_B;
                break;
            } else if (c == 'n' || c == 'N') {
                Serial.println(c);
                hasInverter = false;
                Serial.println(F("OK: Situation B wird uebersprungen."));
                Serial.println(F("Nur Bezug-Analyse (alle Werte unsigned)."));
                Serial.println();
                scanState = STATE_ANALYSE;
                runAnalysis();
                break;
            }
            // andere Zeichen (CR, LF, Leerzeichen) ignorieren
        }
    }
}

// ============================================================================
// SML FRAME PARSER
// ============================================================================
void parseSML(byte b) {
    if (!smlStartFound) {
        if (b == SML_START[startMatchCount]) {
            if (++startMatchCount == 8) {
                smlStartFound   = true;
                bufferPos       = 0;
                startMatchCount = 0;
                endMatchCount   = 0;
                endTrailCount   = 0;
                endEscFound     = false;
                endMarkerFound  = false;
            }
        } else {
            startMatchCount = (b == SML_START[0]) ? 1 : 0;
        }
        return;
    }

    if (bufferPos < SML_BUFFER_SIZE) smlBuffer[bufferPos++] = b;

    if (!endEscFound) {
        if (b == 0x1B) { if (++endMatchCount >= 4) endEscFound = true; }
        else endMatchCount = 0;
    } else if (!endMarkerFound) {
        if (b == 0x1A) { endMarkerFound = true; endTrailCount = 0; }
        else { endEscFound = false; endMatchCount = 0; }
    } else {
        if (++endTrailCount >= 3) {
            if (bufferPos >= SML_MIN_FRAME) {
                // Nur frames verarbeiten wenn wir auf einen warten
                if (scanState == STATE_WAIT_FRAME_A || scanState == STATE_WAIT_FRAME_B) {
                    frameReady = true;
                    digitalWrite(LED_BUILTIN, LOW);
                    delay(50);
                    digitalWrite(LED_BUILTIN, HIGH);
                }
            }
            // Reset
            smlStartFound  = false;
            bufferPos      = 0;
            endMatchCount  = 0;
            endTrailCount  = 0;
            endEscFound    = false;
            endMarkerFound = false;
        }
    }

    if (bufferPos >= SML_BUFFER_SIZE - 10) {
        smlStartFound = false; bufferPos = 0;
        endMatchCount = 0; endTrailCount = 0;
        endEscFound = false; endMarkerFound = false;
    }
}

// ============================================================================
// OBIS AUTO-DISCOVERY
// Sucht alle Muster "77 07 xx xx xx xx xx xx" im Frame
// und dekodiert das folgende TLV-Objekt vollständig
// ============================================================================
int discoverOBIS(ObisObject* list, int maxCount) {
    int found = 0;

    for (int i = 0; i < (int)bufferPos - 30 && found < maxCount; i++) {
        // Muster: 0x77 (Listen-Tag) gefolgt von 0x07 (TL OBIS-ID, Länge 7)
        if (smlBuffer[i] != 0x77) continue;
        if (smlBuffer[i + 1] != 0x07) continue;

        ObisObject& o = list[found];
        o.framePos = i;

        // 6-Byte OBIS-Kennzahl
        for (int j = 0; j < 6; j++) o.id[j] = smlBuffer[i + 2 + j];

        // Prüfe ob schon in Liste (doppelter Match vermeiden)
        bool duplicate = false;
        for (int k = 0; k < found; k++) {
            if (memcmp(list[k].id, o.id, 6) == 0) { duplicate = true; break; }
        }
        if (duplicate) continue;

        // Navigiere durch die folgenden TLV-Felder bis zum Value-TL
        // Suche nach Scaler TL (0x52) und Value TL (0x5x)
        // Typische Struktur: +8=StatusTL, dann Unit, Scaler, Value
        // Wir suchen flexibel: erstes 0x52 nach pos+8 = Scaler

        int scalePos = -1;
        int valueTLPos = -1;

        // Suche innerhalb +8 bis +25 nach 0x52 (Scaler TL)
        for (int off = 8; off <= 20; off++) {
            if (i + off >= (int)bufferPos) break;
            if (smlBuffer[i + off] == 0x52) {
                scalePos  = i + off;
                valueTLPos = scalePos + 2;  // direkt nach Scaler TL + Wert-Byte
                break;
            }
        }

        if (scalePos < 0 || valueTLPos + 1 >= (int)bufferPos) continue;

        uint8_t vTL     = smlBuffer[valueTLPos];
        uint8_t vType   = (vTL >> 4) & 0x0F;  // Typ: 5=signed, 6=unsigned
        uint8_t vLenRaw = (vTL & 0x0F);        // Gesamt-Länge inkl. TL-Byte
        if (vLenRaw < 2 || vLenRaw > 9) continue;  // unplausibel
        uint8_t vLen    = vLenRaw - 1;             // Nutzwert-Bytes

        o.valueTL   = vTL;
        o.valueLen  = vLen;
        o.scaler    = (int8_t)smlBuffer[scalePos + 1];
        o.isSigned  = (vType == 5);
        o.foundInB  = false;

        // Rohwert lesen (big-endian, Nutzwert-Bytes ab valueTLPos+1)
        if (valueTLPos + 1 + vLen > (int)bufferPos) continue;
        o.rawA    = readInt64(valueTLPos + 1, vLen);
        o.calcA   = calcValue(o.rawA, o.scaler);
        o.rawB    = 0;
        o.calcB   = 0.0f;

        found++;
    }
    return found;
}

// ============================================================================
// INT64 LESEN (big-endian, sign-extended)
// ============================================================================
int64_t readInt64(int pos, int len) {
    int64_t raw = 0;
    for (int i = 0; i < len; i++) {
        raw = (raw << 8) | smlBuffer[pos + i];
    }
    // Sign-Extension wenn Vorzeichen-Bit gesetzt und len < 8
    if (len < 8 && (smlBuffer[pos] & 0x80)) {
        int64_t mask = ((int64_t)1 << (len * 8)) - 1;
        raw = raw | ~mask;  // Vorzeichen auffüllen
    }
    return raw;
}

// ============================================================================
// WERT BERECHNEN
// ============================================================================
float calcValue(int64_t raw, int8_t scaler) {
    return (float)raw * powf(10.0f, (float)scaler);
}

// ============================================================================
// OBIS-TABELLE AUSGEBEN
// ============================================================================
void printObisTable(ObisObject* list, int count, const char* label) {
    Serial.println();
    Serial.printf("=== %s — %d OBIS-Objekte gefunden ===\n", label, count);
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Nr  OBIS-ID               Pos  TL    Len Scaler  Wert        Typ"));
    Serial.println(F("------------------------------------------------------------"));

    for (int i = 0; i < count; i++) {
        ObisObject& o = list[i];

        char obisStr[22];
        snprintf(obisStr, sizeof(obisStr), "%d-%d:%d.%d.%d*%d",
            o.id[0], o.id[1], o.id[2], o.id[3], o.id[4], o.id[5]);

        // Klassifizierung (Nice-to-have)
        const char* typ = classifyObis(o.id);

        Serial.printf("%2d  %-20s @%-4d 0x%02X  %dB  %4d  %10.2f  %s\n",
            i + 1,
            obisStr,
            o.framePos,
            o.valueTL,
            o.valueLen,
            o.scaler,
            o.calcA,
            typ
        );
    }
    Serial.println();
}

// ============================================================================
// ANALYSE: Rahmen B mit Rahmen A vergleichen, Vorzeichen ableiten
// ============================================================================
void runAnalysis() {
    // Fix 2: Zählerstand aus bestem verfügbaren Frame (B aktueller als A)
    for (int i = 0; i < obisCountA; i++)
        if (obisA[i].id[2] == 0x01 && obisA[i].id[3] == 0x08)
            counterValue = obisA[i].calcA;
    for (int i = 0; i < obisCountB; i++)
        if (obisB[i].id[2] == 0x01 && obisB[i].id[3] == 0x08)
            counterValue = obisB[i].calcA;  // B überschreibt A

    Serial.println(F("\n============================================================"));
    Serial.println(F("  ANALYSE"));
    Serial.println(F("============================================================"));
    Serial.printf("Zaehlerstand  : %.3f kWh\n", counterValue);
    Serial.printf("Situation A   : +%.1f W  (Bezug)\n",  meterValueA);
    if (hasInverter)
        Serial.printf("Situation B   :  %.1f W  (Einspeisung)\n", meterValueB);
    else
        Serial.println(F("Situation B   :  nicht gemessen (kein Inverter)"));
    Serial.println();

    // Frame B-Werte den OBIS-Objekten aus A zuordnen
    for (int i = 0; i < obisCountA; i++) {
        for (int j = 0; j < obisCountB; j++) {
            if (memcmp(obisA[i].id, obisB[j].id, 6) == 0) {
                obisA[i].rawB   = obisB[j].rawA;  // rawA von B ist der B-Wert
                obisA[i].calcB  = obisB[j].calcA;
                obisA[i].foundInB = true;
                break;
            }
        }
    }

    // Tabelle: A-Wert, B-Wert, Differenz
    Serial.println(F("Nr  OBIS-ID               Wert-A      Wert-B      Delta       Beschreibung"));
    Serial.println(F("-------------------------------------------------------------------------------------"));

    for (int i = 0; i < obisCountA; i++) {
        ObisObject& o = obisA[i];
        char obisStr[22];
        snprintf(obisStr, sizeof(obisStr), "%d-%d:%d.%d.%d*%d",
            o.id[0], o.id[1], o.id[2], o.id[3], o.id[4], o.id[5]);

        float delta = o.foundInB ? (o.calcB - o.calcA) : 0.0f;

        Serial.printf("%2d  %-20s %10.2f  %10.2f  %10.2f  %s\n",
            i + 1,
            obisStr,
            o.calcA,
            o.foundInB ? o.calcB : 0.0f,
            delta,
            obisName(o.id).c_str()
        );
    }

    // Auto-Phasen-Detect und Export-Counter
    bool hasL1 = false, hasL2 = false, hasL3 = false;
    int  exportIdx = -1;

    for (int i = 0; i < obisCountA; i++) {
        uint8_t b2 = obisA[i].id[2], b3 = obisA[i].id[3];
        if (b2 == 0x24 && b3 == 0x07) hasL1 = true;
        if (b2 == 0x38 && b3 == 0x07) hasL2 = true;
        if (b2 == 0x4C && b3 == 0x07) hasL3 = true;
        if (b2 == 0x02 && b3 == 0x08) exportIdx = i;
    }

    if (hasL1 && hasL2 && hasL3)
        Serial.println(F("PHASEN: 3-Phasen-Anlage erkannt (L1 + L2 + L3 vorhanden)"));
    else if (hasL1 && !hasL2 && !hasL3)
        Serial.println(F("PHASEN: 1-Phasen-Anlage erkannt (nur L1 vorhanden)"));
    else
        Serial.println(F("PHASEN: Teilweise erkannt — L1/L2/L3 pruefen"));

    if (exportIdx >= 0)
        Serial.printf("EXPORT: 1-0:2.8.0 gefunden @ Frame-Pos %d  Wert: %.3f kWh\n",
            obisA[exportIdx].framePos, obisA[exportIdx].calcA);

    Serial.println();
    // Das OBIS mit dem geringsten Abstand zu meterValueA UND meterValueB
    int bestPow   = -1;
    float bestErr = 1e9f;

    for (int i = 0; i < obisCountA; i++) {
        if (!obisA[i].foundInB) continue;
        float errA = fabsf(obisA[i].calcA - meterValueA);
        float errB = fabsf(obisA[i].calcB - meterValueB);
        float err  = errA + errB;
        if (err < bestErr) {
            bestErr = err;
            bestPow = i;
        }
    }

    Serial.println();
    if (bestPow >= 0) {
        Serial.println(F("-------------------------------------------------------------------------------------"));
        Serial.printf("=> Bester Kandidat für 'pow' (Summe): OBIS %d-%d:%d.%d.%d*%d\n",
            obisA[bestPow].id[0], obisA[bestPow].id[1], obisA[bestPow].id[2],
            obisA[bestPow].id[3], obisA[bestPow].id[4], obisA[bestPow].id[5]);
        Serial.printf("   Abweichung von Zähleranzeige: A=%.1fW, B=%.1fW\n",
            fabsf(obisA[bestPow].calcA - meterValueA),
            fabsf(obisA[bestPow].calcB - meterValueB));
    }

    generateCode();
}

// ============================================================================
// CODE-GENERIERUNG
// ============================================================================
void generateCode() {
    Serial.println(F("\n============================================================"));
    Serial.println(F("  GENERIERTER CODE"));
    Serial.println(F("  Einfach in den Production-Sketch kopieren"));
    Serial.println(F("============================================================\n"));

    if (!hasInverter) {
        Serial.println(F("// HINWEIS: Nur Situation A gemessen (kein Inverter)."));
        Serial.println(F("// Vorzeichen der Leistungswerte unbekannt."));
        Serial.println(F("// Alle Werte werden als unsigned ausgegeben."));
        Serial.println(F("// Fuer Vorzeichen-Erkennung Scanner mit Inverter wiederholen.\n"));
    }

    // OBIS-Arrays
    Serial.println(F("// OBIS-Codes (aus SML-Scanner ermittelt)"));
    Serial.println(F("// Format: 77 07 + 6-Byte OBIS-Kennzahl"));

    for (int i = 0; i < obisCountA; i++) {
        ObisObject& o = obisA[i];

        // Variablenname ableiten
        char varName[30];
        String name = obisName(o.id);
        if      (o.id[2]==0x10 && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERSUM");
        else if (o.id[2]==0x24 && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERL1");
        else if (o.id[2]==0x38 && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERL2");
        else if (o.id[2]==0x4C && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERL3");
        else if (o.id[2]==0x01 && o.id[3]==0x08) snprintf(varName, 30, "OBISCOUNTER");
        else if (o.id[2]==0x02 && o.id[3]==0x08) snprintf(varName, 30, "OBISCOUNTER_OUT");
        else snprintf(varName, 30, "OBIS_%02X%02X", o.id[2], o.id[3]);

        Serial.printf("const byte %-16s[] = {0x77, 0x07, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};  // %s\n",
            varName,
            o.id[0], o.id[1], o.id[2], o.id[3], o.id[4], o.id[5],
            name.c_str()
        );
    }

    // Value-Offsets
    Serial.println();
    Serial.println(F("// Value-Offsets (aus SML-Scanner ermittelt)"));
    Serial.println(F("// ScalerOffset = Position des Scaler-Bytes ab OBIS-Start (+0=0x77)"));
    Serial.println(F("// ValueOffset  = Position des ersten Nutzwert-Bytes ab OBIS-Start"));
    Serial.println(F("// ValueLen     = Anzahl Nutzwert-Bytes"));
    Serial.println();

    Serial.println(F("float readObisValue(int pos, int scalerOffset, int valueOffset, int valueLen) {"));
    Serial.println(F("    if (pos + valueOffset + valueLen >= bufferPos) return 0.0f;"));
    Serial.println(F("    if (smlBuffer[pos + scalerOffset - 1] != 0x52) return 0.0f;  // Scaler TL"));
    Serial.println(F("    int8_t scaler = (int8_t)smlBuffer[pos + scalerOffset];"));
    Serial.println(F("    // Value TL validieren"));
    Serial.println(F("    // uint8_t vTL = smlBuffer[pos + valueOffset - 1];"));
    Serial.println(F("    int64_t raw = 0;"));
    Serial.println(F("    for (int i = 0; i < valueLen; i++)"));
    Serial.println(F("        raw = (raw << 8) | smlBuffer[pos + valueOffset + i];"));
    Serial.println(F("    // Sign-Extension"));
    Serial.println(F("    if (valueLen < 8 && (smlBuffer[pos + valueOffset] & 0x80))"));
    Serial.println(F("        raw |= ~(((int64_t)1 << (valueLen * 8)) - 1);"));
    Serial.println(F("    return (float)raw * powf(10.0f, (float)scaler);"));
    Serial.println(F("}\n"));

    // Konkrete Aufrufe
    Serial.println(F("// Konkrete Aufrufe in decodeSML():"));
    for (int i = 0; i < obisCountA; i++) {
        ObisObject& o = obisA[i];

        // ScalerOffset berechnen: Position des Scaler-Bytes ab OBIS-Start
        // Wir suchen 0x52 ab +8
        int scalerOff = -1;
        for (int off = 8; off <= 20; off++) {
            if (o.framePos + off < (int)bufferPos && smlBuffer[o.framePos + off] == 0x52) {
                scalerOff = off + 1;  // +1 für das Wert-Byte nach TL
                break;
            }
        }
        if (scalerOff < 0) continue;

        int valueTLOff = scalerOff + 1;  // Value TL direkt nach Scaler-Wert
        int valueOff   = valueTLOff + 1; // Erstes Nutzwert-Byte

        char varName[30];
        if      (o.id[2]==0x10 && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERSUM");
        else if (o.id[2]==0x24 && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERL1");
        else if (o.id[2]==0x38 && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERL2");
        else if (o.id[2]==0x4C && o.id[3]==0x07) snprintf(varName, 30, "OBISPOWERL3");
        else if (o.id[2]==0x01 && o.id[3]==0x08) snprintf(varName, 30, "OBISCOUNTER");
        else if (o.id[2]==0x02 && o.id[3]==0x08) snprintf(varName, 30, "OBISCOUNTER_OUT");
        else snprintf(varName, 30, "OBIS_%02X%02X", o.id[2], o.id[3]);

        Serial.printf("// %s  TL=0x%02X (%dB)  Scaler@+%d  Value@+%d\n",
            obisName(o.id).c_str(), o.valueTL, o.valueLen, scalerOff, valueOff);
        Serial.printf("pos = findOBIS(%-16s, 8);\n", varName);
        Serial.printf("if (pos >= 0) val = readObisValue(pos, %d, %d, %d);\n\n",
            scalerOff, valueOff, o.valueLen);
    }

    Serial.println(F("============================================================"));
    Serial.println(F("  Scanner abgeschlossen. ESP neu starten fuer neuen Scan."));
    Serial.println(F("============================================================"));
}

// ============================================================================
// SERIAL LINE LESEN (non-blocking)
// ============================================================================
bool readSerialLine(float &result) {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialInputPos > 0) {
                serialInputBuf[serialInputPos] = '\0';
                result = atof(serialInputBuf);
                serialInputPos = 0;
                Serial.println(serialInputBuf);
                return true;
            }
        } else if (serialInputPos < 18) {
            serialInputBuf[serialInputPos++] = c;
        }
    }
    return false;
}

// ============================================================================
// OBIS-KLASSIFIZIERUNG (Nice-to-have: Typ-Marker für Tabelle)
// ============================================================================
const char* classifyObis(byte* id) {
    uint8_t b2 = id[2], b3 = id[3];

    // Energie-Zähler
    if (b3 == 0x08) return "[ZAEHLER]";

    // Leistung Summe
    if ((b2 == 0x10 || b2 == 0x0F) && b3 == 0x07) return "[POW-SUM] *";

    // Leistung Phasen (auto-detect L1/L2/L3)
    if (b2 == 0x24 && b3 == 0x07) return "[POW-L1]  *";
    if (b2 == 0x38 && b3 == 0x07) return "[POW-L2]  *";
    if (b2 == 0x4C && b3 == 0x07) return "[POW-L3]  *";

    // Einspeisung Export-Zähler (Nice-to-have)
    if (b2 == 0x02 && b3 == 0x08) return "[EXPORT-ZAEHLER]";

    // Blindleistung Phasen
    if (b2 == 0x23 && b3 == 0x07) return "[BLIND-L1]";
    if (b2 == 0x37 && b3 == 0x07) return "[BLIND-L2]";
    if (b2 == 0x4B && b3 == 0x07) return "[BLIND-L3]";

    // Spannung (Nice-to-have: markieren)
    if (b2 == 0x20 && b3 == 0x07) return "[VOLT-L1]";
    if (b2 == 0x34 && b3 == 0x07) return "[VOLT-L2]";
    if (b2 == 0x48 && b3 == 0x07) return "[VOLT-L3]";

    // Strom (Nice-to-have: markieren)
    if (b2 == 0x1F && b3 == 0x07) return "[AMP-L1]";
    if (b2 == 0x33 && b3 == 0x07) return "[AMP-L2]";
    if (b2 == 0x47 && b3 == 0x07) return "[AMP-L3]";

    // Geräte-ID
    if (b2 == 0x60 && b3 == 0x01) return "[GERAETE-ID]";

    return "[?]";
}

// ============================================================================
// HILFSFUNKTIONEN: OBIS-NAME NACHSCHLAGEN
// ============================================================================
String obisName(byte* id) {
    // Nur Byte 2+3 relevant für Identifikation
    uint8_t b2 = id[2], b3 = id[3];

    if (b2 == 0x01 && b3 == 0x08) return F("Wirkenergie Bezug (1-0:1.8.0)");
    if (b2 == 0x02 && b3 == 0x08) return F("Wirkenergie Einspeisung (1-0:2.8.0)");
    if (b2 == 0x01 && b3 == 0x09) return F("Blindenergie ind. Bezug (1-0:1.9.0)");
    if (b2 == 0x02 && b3 == 0x09) return F("Blindenergie ind. Einsp. (1-0:2.9.0)");
    if (b2 == 0x10 && b3 == 0x07) return F("Wirkleistung Summe (1-0:16.7.0)");
    if (b2 == 0x0F && b3 == 0x07) return F("Wirkleistung Betrag (1-0:15.7.0)");
    if (b2 == 0x24 && b3 == 0x07) return F("Wirkleistung L1 (1-0:36.7.0)");
    if (b2 == 0x38 && b3 == 0x07) return F("Wirkleistung L2 (1-0:56.7.0)");
    if (b2 == 0x4C && b3 == 0x07) return F("Wirkleistung L3 (1-0:76.7.0)");
    if (b2 == 0x20 && b3 == 0x07) return F("Spannung L1 (1-0:32.7.0)");
    if (b2 == 0x34 && b3 == 0x07) return F("Spannung L2 (1-0:52.7.0)");
    if (b2 == 0x48 && b3 == 0x07) return F("Spannung L3 (1-0:72.7.0)");
    if (b2 == 0x1F && b3 == 0x07) return F("Strom L1 (1-0:31.7.0)");
    if (b2 == 0x33 && b3 == 0x07) return F("Strom L2 (1-0:51.7.0)");
    if (b2 == 0x47 && b3 == 0x07) return F("Strom L3 (1-0:71.7.0)");
    if (b2 == 0x23 && b3 == 0x07) return F("Blindleistung L1 (1-0:35.7.0)");
    if (b2 == 0x37 && b3 == 0x07) return F("Blindleistung L2 (1-0:55.7.0)");
    if (b2 == 0x4B && b3 == 0x07) return F("Blindleistung L3 (1-0:75.7.0)");
    if (b2 == 0x60 && b3 == 0x01) return F("Geraete-ID / Seriennummer");
    if (b2 == 0x60 && b3 == 0x07) return F("Aktuelle Leistung gesamt");

    char buf[30];
    snprintf(buf, sizeof(buf), "Unbekannt (%02X.%02X)", b2, b3);
    return String(buf);
}
