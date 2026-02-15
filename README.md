# SML Universal IR-Scanner für Smartmeter — automatiche Protokollanalyse - Programmierunterstützung

**Firmware:** sml_universal_scanner v1.4  
**Hardware:** Wemos D1 Mini o.ä. (ESP8266) + Hichi IR-Lesekopf o.ä. (IR/TTL)  
**Zweck:** Unbekannte SML-Smartmeter mit Infrarot-Augabe analysieren und C++ -Code zur Auswertung generieren  
**Datum:** Februar 2026  

---

## 1. Konzept

Das Scann-Programm dient zur automatisierten Analyse des SML- Protokoll (Smart Message Language) 
von Smartmetern mit Infrarot-Ausgabe. 

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

Nach Scannzyklus Bezug (A) fragt der Scanner ob ein Einspeiseinverter vorhanden ist:

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

Aus der Entwicklung gewonnene Erkenntnisse,
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

## 13. MQTT Minimal-Sketch — Beispiel Sketch mit MQTT-Übertragung der Smartmeter-Daten

Der Sketch (mqtt_minimal.ino) ist nach Anpassungen in den zwei markierten Blöcken (Zugangsdaten, Scanner-Output)
direkt compilierbar und lauffähig.

---

### Workflow: MQTT Minimal-Sketch

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

5. OBIS-Arrays  → aus "SCHRITT 2 / OBIS-Sequenzen" einfügen
   #define-Werte → Lese-Parameter eintragen

6. WLAN/MQTT    → in "SCHRITT 1" anpassen
7. MQTT Minimal-Sketch flashen → fertig
```

### MQTT-Ausgabe (Beispiel)

```
Topic: meter/power

   Leistungswerte:
   
  {"pow":-416.0,"L1":-696.6,"L2":15.8,"L3":264.8}

Topic: meter/counter

   Zählerstand:

  {"kwh_in":5289.803,"kwh_out":1842.640}

Topic: meter/status

   Statusmaeldung:

  "online"   (retained, LWT: "offline")
```
