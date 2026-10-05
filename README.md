# ESP32-Quadrocopter

Selbstgebaute Drohne mit selbstprogrammierter Flugsteuerung und eigener Fernsteuerung.
Beide Seiten basieren auf einem ESP32 und kommunizieren über ESP-NOW.

Entstanden im Rahmen des Projektseminars 1 (EIT-F-500) an der THM.

## Aufbau

| | Drohne | Fernsteuerung |
|---|---|---|
| Controller | ESP32 | ESP32 |
| Sensorik | MPU6050 (I²C) | – |
| Bedienung | – | 2 Joysticks, 2 OLED-Displays (SSD1306) |
| Antrieb | 4 × YSIDO 1204, 6000 KV | – |
| Versorgung | LiPo 2S–3S | 18650-Zelle + Step-up auf 5 V |
| Rahmen | PLA, 3D-gedruckt | PLA, 3D-gedruckt |

## Software der Flugsteuerung

- **Kaskadenregelung** pro Achse: äußerer Winkelregler (KP_ANGLE = 4,0, Soll-Drehrate begrenzt auf ±250 °/s), innerer PID-Regler auf die Drehrate
- **Komplementärfilter** aus Gyroskop und Beschleunigungssensor, Zeitkonstante 1 s
- **PT1-Tiefpass** auf der gemessenen Drehrate, damit der D-Anteil nicht auf Sensorrauschen reagiert
- **Airmode-Mixer**: skaliert die Mischwerte und verschiebt das Grundgas, damit die Drohne auch bei niedrigem Gas steuerbar bleibt
- **Yaw-Heading-Hold**: hält die zuletzt angesteuerte Richtung, sobald der Yaw-Stick losgelassen wird
- **FreeRTOS**: Regelkreis mit 1 kHz auf Kern 1, Akkuüberwachung und Telemetrie auf Kern 0

### Sicherheitsfunktionen

- Scharfschalten nur bei zentrierten Sticks (Toleranz 400 Rohwerte) und Gas unter 1220
- Start wird verweigert, wenn der MPU6050 nicht antwortet oder ESP-NOW nicht initialisiert — die Motoren bleiben dann dauerhaft aus
- Failsafe nach 500 ms ohne Funkpaket: Selbst-Entschärfung, Sticks auf Mitte
- Sanfte Gasrampe beim Entschärfen (0,4 Einheiten/ms) statt hartem Abschalten
- Notstopp gegen Integrator-Windup: Bleibt die Reglerausgabe zu lange über 350, schalten die Motoren sofort ab
- Empfangene Pakete werden auf Länge geprüft und in einer kritischen Sektion übernommen

## Ordner

```
flugcontroller_V10_6/   Flugsteuerung (Drohne)
DroneSender_V2/         Fernsteuerung
archiv/                 ältere Programmstände
cad/                    3D-Modelle für Rahmen und Gehäuse (3MF)
bericht/                Projektbericht als PDF
```

## Aufspielen

Arduino IDE mit installiertem ESP32-Board-Support. Benötigte Bibliotheken:

- **Flugsteuerung:** `MPU6050`, `QuickPID`
- **Fernsteuerung:** `Adafruit_GFX`, `Adafruit_SSD1306`

`esp_now.h`, `WiFi.h` und `Wire.h` kommen mit dem ESP32-Paket.

> **Wichtig:** In `DroneSender_V2.ino` steht die MAC-Adresse der Drohne fest im Code
> (`droneAddress`). Beim Nachbau muss dort die Adresse des eigenen ESP32 eingetragen
> werden, sonst kommen die Steuerpakete nirgends an.

### Pinbelegung Drohne

| Funktion | GPIO |
|---|---|
| Motoren FL / FR / BL / BR | 32 / 25 / 27 / 26 |
| I²C (SDA / SCL) | 21 / 23 |
| Akkumessung | 34 |

## Stand und Ausblick

Die Drohne fliegt stabil und hält die Lage. Was fehlt, ist ein Bezug zur Umgebung:
Ohne Positionsbestimmung driftet sie langsam weg und kann keine feste Höhe halten.

Geplante Erweiterung:

- **VL53L1X** — Laser-Abstandssensor für die Höhe
- **BMP580** — Barometer für Höhen über die Laserreichweite hinaus
- **PMW3901** — optischer Flusssensor gegen das seitliche Driften
