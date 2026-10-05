#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

// =====================================================================
// DroneSender V2
//
// Die Gas-Logik bleibt exakt wie gehabt:
//   Stick in der Mitte -> Prozentwert bleibt stehen
//   Stick nach oben    -> Prozentwert steigt
//   Stick nach unten   -> Prozentwert faellt
//
// [FIX 1] Gasrampe ist jetzt zeitbasiert (Prozent pro Sekunde) statt
//         "2.0 pro Schleifendurchlauf". Vorher hing die Rampengeschwin-
//         digkeit an der tatsaechlichen Loop-Zeit und ruckelte, sobald
//         die Displays aktualisiert wurden.
//
// [FIX 2] Stickmitten werden beim Start automatisch gemessen, statt 1808
//         fuer alle vier Achsen fest zu verdrahten. Plausibilitaets-
//         pruefung mit Rueckfall auf 1808.
//
// [FIX 3] Alle Achsen werden nach dem map() auf 1000..2000 begrenzt.
//
// [FIX 4] display.begin() wird geprueft. Schlaegt die Pufferallokation
//         fehl, fuehrte clearDisplay() vorher auf einen Nullpointer.
//
// [FIX 5] Telemetrie-Timeout: ohne Antwort der Drohne wird "--" gezeigt,
//         statt den letzten Wert einzufrieren.
//
// [FIX 6] ADC wird gemittelt und mit Dummy-Read gelesen (ESP32-Erratum
//         fuer GPIO36/VP: erste Messung nach Kanalwechsel kann falsch sein).
//
// [FIX 7] Armen nur, wenn der Gas-Stick in der Mitte steht.
//
// [FIX 8] Feste Sendrate von 20 ms, unabhaengig von den Displays.
// =====================================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_1, -1);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_2, -1);
bool disp1_ok = false;
bool disp2_ok = false;

// ---------------------------------------------------------------------
// PINS
// ---------------------------------------------------------------------
const int PIN_LEFT_YAW      = 34;
const int PIN_LEFT_THROTTLE = 35;
const int PIN_RIGHT_ROLL    = 36;   // VP
const int PIN_BATTERY       = 32;
const int PIN_RIGHT_PITCH   = 33;
const int PIN_BTN_ARM       = 13;
const int PIN_CALIB_SW      = 4;    // Rechter Joystick-Klick

uint8_t droneAddress[] = { 0x68, 0x09, 0x47, 0x9D, 0xFA, 0x4C };

// ---------------------------------------------------------------------
// DATENSTRUKTUREN
// ---------------------------------------------------------------------
typedef struct struct_cmd {
  int  throttle;
  int  yaw;
  int  roll;
  int  pitch;
  bool isArmed;
  bool triggerCalib;
} struct_cmd;

// [FIX 19] MUSS exakt zur Struktur in der Drohne passen. Passt sie nicht,
// verwirft der Empfaenger das Paket wegen falscher Laenge - also immer
// beide Seiten zusammen flashen.
typedef struct struct_telemetry {
  float drone_voltage;
  int   mpu_status;
  float gyro_drift;
  float mpu_temp;
} struct_telemetry;

struct_cmd       myData      = { 1000, 1500, 1500, 1500, false, false };
struct_telemetry myTelemetry = { 0.0f, 0 };

// ---------------------------------------------------------------------
// SYSTEM
// ---------------------------------------------------------------------
bool isConnected = false;
bool isArmed     = false;
bool lastBtnState = HIGH;
unsigned long lastDebounceTime  = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSendTime      = 0;
volatile unsigned long lastRecvTime = 0;
unsigned long lastRampMicros = 0;

const unsigned long SEND_INTERVAL_MS      = 20;
const unsigned long DISPLAY_INTERVAL_MS   = 100;
const unsigned long TELEMETRY_TIMEOUT_MS  = 1000;

// ---------------------------------------------------------------------
// AKKU (Controller)
// ---------------------------------------------------------------------
int final_ctrl_percent = -1;
// 3.95V (Multimeter) - 3.72V (ESP-Berechnung) = 0.23V Verlust durch Kabel und Akku-Alter
const float VOLTAGE_COMPENSATION = 0.23f;

// ---------------------------------------------------------------------
// VIRTUELLER GAS-STICK
// ---------------------------------------------------------------------
const int   THROTTLE_MIN = 1180;   // muss zu MIN_THROTTLE der Drohne passen
const int   THROTTLE_MAX = 2000;
// [FIX 9] Puffer um die Gas-Stickmitte, in ADC-Rohwerten.
//
// WARUM DAS GROESSER IST ALS FRUEHER:
// In V1 war die Mitte fest auf 1808 verdrahtet, obwohl der Stick in Ruhe
// woanders liegt. Der Puffer war dadurch UNSYMMETRISCH - nach unten hin
// gab es mehr Luft als nach oben. Genau diese Extra-Luft hat das kurze
// Ueberschwingen beim Loslassen des Sticks geschluckt.
// V2 misst die echte Mitte und legt den Puffer symmetrisch drumherum.
// Damit war die Reserve nach unten ploetzlich kleiner und der
// Ueberschwinger wurde als echter "runter"-Befehl gewertet.
const int THROTTLE_DEADBAND = 450;

// [FIX 9] Ueberschwing-Sperre. Ein federnder Stick schiesst beim
// Loslassen kurz ueber die Mitte hinaus. Innerhalb dieser Zeit nach der
// letzten aktiven Auslenkung wird die GEGENRICHTUNG ignoriert.
// Willst du bewusst direkt von hoch auf runter, kurz warten (oder Wert
// kleiner machen).
const unsigned long RELEASE_LOCKOUT_MS = 200;

// [FIX 9] Falls du lieber die alte feste Mitte willst: auf false setzen.
const bool USE_MEASURED_CENTER = true;

// [FIX 1] Zeitbasiert. 100.0 entsprach dem alten Verhalten (2.0 pro 20 ms).
//
// [FIX 18] Auf 200 angehoben. Mit 100 dauerte es 2.7 s vom Leerlauf bis
// zum Schwebegas - die ganze Zeit steht die Drohne im Bodeneffekt und
// kann sich auf ihren Beinen aufschaukeln. Mit 200 sind es 1.3 s.
// Zu traege? Hoeher. Zu zappelig beim Dosieren? Wieder auf 100.
const float THROTTLE_RATE_PER_S = 200.0f;

float currentThrottle = 1000.0f;

// =====================================================================
// [FIX 11] STICKRICHTUNGEN - hier und NUR hier drehst du eine Achse um.
//
// Vorher standen die Umkehrungen als "3000 - wert" mitten im Code
// verteilt. Wenn eine Achse falsch herum reagiert, aendere genau eine
// Zeile hier, nicht die Vorzeichen im Mixer der Drohne.
//
// Gemessen am 01.09.: Stick nach vorne liess die Drohne nach hinten
// gehen -> INVERT_PITCH von true auf false.
// =====================================================================
const bool INVERT_ROLL  = false;
const bool INVERT_PITCH = false;   // war true, deshalb war vorne/hinten vertauscht
const bool INVERT_YAW   = true;

// [FIX 2] Gemessene Stickmitten
const int CENTER_FALLBACK = 1808;
int center_yaw   = CENTER_FALLBACK;
int center_roll  = CENTER_FALLBACK;
int center_pitch = CENTER_FALLBACK;
int center_thr   = CENTER_FALLBACK;

// =====================================================================
// HILFSFUNKTIONEN
// =====================================================================

// [FIX 6] Dummy-Read + Mittelung. Deckt das GPIO36/39-Erratum ab und
// glaettet gleichzeitig das ADC-Rauschen.
// Hinweis: bewusst OHNE Default-Argument. Die automatische Prototyp-
// Erzeugung der Arduino-IDE kopiert Defaults mit und erzeugt dann den
// Fehler "default argument given for parameter".
int readAdc(int pin, int samples) {
  analogRead(pin);              // Dummy: erste Messung nach Kanalwechsel verwerfen
  long sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return (int)(sum / samples);
}

// [FIX 2] + [FIX 3] Mitte exakt auf 1500 zwingen, Ergebnis begrenzen.
int mapStick(int raw, int center) {
  center = constrain(center, 500, 3500);   // schuetzt map() vor Division durch 0
  int v;
  if (raw < center) v = map(raw, 0, center, 1000, 1500);
  else              v = map(raw, center, 4095, 1500, 2000);
  return constrain(v, 1000, 2000);
}

// [FIX 15] Defekte Achse blockiert das Armieren.
//
// Vorher wurde eine unplausible Ruhelage nur auf Serial gemeldet und
// still auf 1808 zurueckgesetzt - der Controller liess sich trotzdem
// scharfschalten. Am 02.09. hat genau das eine gebrochene Poti-Zuleitung
// verschleiert: Roll lag konstant auf Rohwert ~15, die Drohne bekam einen
// Dauerbefehl von -16 Grad und war nicht mehr steuerbar.
bool stick_fault = false;
char stick_fault_name[10] = "";

int measureCenter(int pin, const char *name) {
  int v = readAdc(pin, 32);
  // Plausibel? Ein Joystick in Ruhelage liegt grob in der Mitte des Bereichs.
  if (v < 1400 || v > 2400) {
    Serial.printf("FEHLER Achse %s (Pin %d): Ruhewert %d, erwartet 1400..2400.\n", name, pin, v);
    Serial.println("       Nahe 0 oder 4095 deutet auf eine gebrochene Poti-Leitung hin.");
    Serial.println("       Armieren ist gesperrt, bis das behoben und neu gestartet ist.");
    stick_fault = true;
    strncpy(stick_fault_name, name, sizeof(stick_fault_name) - 1);
    stick_fault_name[sizeof(stick_fault_name) - 1] = '\0';
    return CENTER_FALLBACK;
  }
  return v;
}

// =====================================================================
// ESP-NOW CALLBACKS
// =====================================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  isConnected = (status == ESP_NOW_SEND_SUCCESS);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len != (int)sizeof(myTelemetry)) return;
  memcpy(&myTelemetry, incomingData, sizeof(myTelemetry));
  lastRecvTime = millis();
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PIN_LEFT_YAW,      INPUT);
  pinMode(PIN_LEFT_THROTTLE, INPUT);
  pinMode(PIN_RIGHT_ROLL,    INPUT);
  pinMode(PIN_RIGHT_PITCH,   INPUT);

  pinMode(PIN_BTN_ARM,  INPUT_PULLUP);
  pinMode(PIN_CALIB_SW, INPUT_PULLUP);

  I2C_1.begin(21, 22, 100000);
  I2C_2.begin(18, 19, 100000);

  // [FIX 4] Rueckgabewert pruefen, sonst spaeter Nullpointer.
  disp1_ok = display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  disp2_ok = display2.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!disp1_ok) Serial.println("WARNUNG: Display 1 nicht gefunden.");
  if (!disp2_ok) Serial.println("WARNUNG: Display 2 nicht gefunden.");

  // -------------------------------------------------------------------
  // [FIX 2] Stickmitten messen (Joysticks dabei NICHT anfassen)
  // -------------------------------------------------------------------
  delay(200);
  if (USE_MEASURED_CENTER) {
    center_yaw   = measureCenter(PIN_LEFT_YAW,      "YAW");
    center_thr   = measureCenter(PIN_LEFT_THROTTLE, "GAS");
    center_roll  = measureCenter(PIN_RIGHT_ROLL,    "ROLL");
    center_pitch = measureCenter(PIN_RIGHT_PITCH,   "PITCH");
  }
  Serial.printf("Mitten: yaw=%d thr=%d roll=%d pitch=%d (alt fest: %d)\n",
                center_yaw, center_thr, center_roll, center_pitch, CENTER_FALLBACK);
  Serial.printf("Gas-Fenster: runter unter %d, hoch ueber %d\n",
                center_thr - THROTTLE_DEADBAND, center_thr + THROTTLE_DEADBAND);

  // -------------------------------------------------------------------
  // AKKU-MESSUNG (Ruhespannung, einmalig beim Start)
  // -------------------------------------------------------------------
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(PIN_BATTERY);
    delay(2);
  }
  float avg_raw = sum / 50.0f;
  float ruhespannung = ((avg_raw / 4095.0f) * 3.3f * 2.0f * 1.08f) + VOLTAGE_COMPENSATION;

  // 3.5V = 0%, 4.2V = 100%
  final_ctrl_percent = map((int)(ruhespannung * 100), 350, 420, 0, 100);
  final_ctrl_percent = constrain(final_ctrl_percent, 0, 100);

  Serial.print("Kompensierte Spannung: "); Serial.println(ruhespannung);
  Serial.print("Initialer Akku: ");        Serial.println(final_ctrl_percent);

  // -------------------------------------------------------------------
  // FUNK
  // -------------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("FEHLER: ESP-NOW Init fehlgeschlagen.");
    if (disp1_ok) {
      display1.clearDisplay();
      display1.setTextSize(1);
      display1.setTextColor(SSD1306_WHITE);
      display1.setCursor(0, 0);
      display1.print("ESP-NOW FEHLER");
      display1.display();
    }
    for (;;) delay(1000);
  }

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
      esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  #else
      esp_now_register_send_cb(OnDataSent);
      esp_now_register_recv_cb(OnDataRecv);
  #endif

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, droneAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("FEHLER: Peer konnte nicht angelegt werden.");
  }

  lastRampMicros = micros();
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  unsigned long now = millis();

  // ---------- Rohwerte ----------
  int rawThrottle = readAdc(PIN_LEFT_THROTTLE, 4);
  int read_yaw    = readAdc(PIN_LEFT_YAW, 4);
  int read_roll   = readAdc(PIN_RIGHT_ROLL, 4);
  int read_pitch  = readAdc(PIN_RIGHT_PITCH, 4);

  bool throttleStickCentered = (rawThrottle <= center_thr + THROTTLE_DEADBAND) &&
                               (rawThrottle >= center_thr - THROTTLE_DEADBAND);

  // [FIX 15] Auch Roll, Pitch und Yaw muessen zum Armieren in Ruhe stehen.
  const int STICK_ARM_TOLERANCE = 400;   // Rohwerte
  bool allSticksCentered = throttleStickCentered &&
      (abs(read_roll  - center_roll)  < STICK_ARM_TOLERANCE) &&
      (abs(read_pitch - center_pitch) < STICK_ARM_TOLERANCE) &&
      (abs(read_yaw   - center_yaw)   < STICK_ARM_TOLERANCE);

  // ---------- ARMING ----------
  int btnState = digitalRead(PIN_BTN_ARM);
  if (btnState == LOW && lastBtnState == HIGH && (now - lastDebounceTime) > 200) {
    if (isArmed) {
      // Disarm ist immer erlaubt (Notaus)
      isArmed = false;
      currentThrottle = 1000.0f;
      lastDebounceTime = now;
    } else if (stick_fault) {
      // [FIX 15] Defekte Achse beim Start erkannt -> gar nicht erst scharf
      Serial.printf("ARM abgelehnt: Achse %s defekt. Verkabelung pruefen.\n", stick_fault_name);
      lastDebounceTime = now;
    } else if (!allSticksCentered) {
      // [FIX 15] Alle vier Achsen muessen in Ruhe stehen, nicht nur Gas.
      // Faengt auch eine Leitung, die erst nach dem Booten abreisst.
      Serial.printf("ARM abgelehnt: Stick nicht mittig (thr:%d roll:%d pitch:%d yaw:%d).\n",
                    rawThrottle, read_roll, read_pitch, read_yaw);
      lastDebounceTime = now;
    } else {
      // [FIX 7] Armen nur mit Sticks in der Mitte
      isArmed = true;
      currentThrottle = (float)THROTTLE_MIN;
      lastDebounceTime = now;
    }
  }
  lastBtnState = btnState;

  // ---------- KALIBRIERUNG ----------
  myData.triggerCalib = (!isArmed && digitalRead(PIN_CALIB_SW) == LOW);

  // ---------- [FIX 1] GAS: virtueller Stick, zeitbasiert ----------
  unsigned long nowUs = micros();
  float dt = (nowUs - lastRampMicros) / 1000000.0f;
  lastRampMicros = nowUs;
  // [FIX 9] Enger deckeln als vorher (war 0.2 s). Ein Display-Update
  // blockiert die Schleife rund 25 ms - ohne engen Deckel wuerde diese
  // ganze Pause der Stickposition zugerechnet, die erst danach gemessen
  // wurde. Bei 0.05 s kann ein Haenger hoechstens 0.6 % ausmachen.
  if (dt > 0.05f) dt = 0.05f;

  // ---------- Richtung bestimmen ----------
  int stickDir = 0;
  if (rawThrottle > center_thr + THROTTLE_DEADBAND)      stickDir = +1;
  else if (rawThrottle < center_thr - THROTTLE_DEADBAND) stickDir = -1;

  // [FIX 9] Ueberschwing-Sperre: Gegenrichtung kurz nach der letzten
  // aktiven Auslenkung verwerfen. Das ist der Grund, warum der Prozent-
  // wert nach dem Loslassen um ein bis zwei Prozent zurueckgefallen ist.
  static int           lastActiveDir = 0;
  static unsigned long lastActiveMs  = 0;

  if (stickDir != 0) {
    if (stickDir == -lastActiveDir && (now - lastActiveMs) < RELEASE_LOCKOUT_MS) {
      stickDir = 0;                 // Ueberschwinger, kein echter Befehl
    } else {
      lastActiveDir = stickDir;
      lastActiveMs  = now;
    }
  }

  if (isArmed) {
    float step = THROTTLE_RATE_PER_S * dt;
    if (stickDir > 0)      currentThrottle += step;
    else if (stickDir < 0) currentThrottle -= step;
    // Stick in der Mitte -> currentThrottle bleibt unveraendert stehen
    currentThrottle = constrain(currentThrottle, (float)THROTTLE_MIN, (float)THROTTLE_MAX);
  } else {
    currentThrottle = 1000.0f;
    lastActiveDir = 0;
  }

  // [FIX 10] Diagnose fuer das langsame Wegdriften des Gaswerts.
  // Zeigt, wie weit der Stick nach dem Loslassen wirklich von der beim
  // Boot gemessenen Mitte entfernt liegt. Ist "Abstand" nahe am Fenster-
  // rand, muss THROTTLE_DEADBAND groesser. Auf 0 setzen, wenn es nervt.
#define DEBUG_THROTTLE 1
#if DEBUG_THROTTLE
  static unsigned long lastThrDbg = 0;
  if (now - lastThrDbg > 500) {
    lastThrDbg = now;
    // [FIX 14] Alle vier Achsen als "IST/MITTE". Wenn eine Achse im
    // Ruhezustand weit von ihrer Mitte weg liegt oder sich beim Bewegen
    // des Sticks kaum aendert, ist der Joystick mechanisch defekt -
    // dann ist es kein Reglerproblem.
    Serial.printf("thr:%4d/%4d  roll:%4d/%4d  pitch:%4d/%4d  yaw:%4d/%4d | GasRand:%4d Dir:%+d Gas:%4d\n",
                  rawThrottle, center_thr,
                  read_roll,   center_roll,
                  read_pitch,  center_pitch,
                  read_yaw,    center_yaw,
                  THROTTLE_DEADBAND - abs(rawThrottle - center_thr),
                  stickDir, (int)currentThrottle);
  }
#endif

  // ---------- PAKET BEFUELLEN ----------
  myData.throttle = (int)currentThrottle;

  int mapped_yaw   = mapStick(read_yaw,   center_yaw);
  int mapped_roll  = mapStick(read_roll,  center_roll);
  int mapped_pitch = mapStick(read_pitch, center_pitch);

  // [FIX 11] Umkehrung zentral ueber die Schalter oben (1000 wird zu 2000,
  // 1500 bleibt 1500).
  myData.roll  = INVERT_ROLL  ? (3000 - mapped_roll)  : mapped_roll;
  myData.pitch = INVERT_PITCH ? (3000 - mapped_pitch) : mapped_pitch;
  myData.yaw   = INVERT_YAW   ? (3000 - mapped_yaw)   : mapped_yaw;

  myData.isArmed = isArmed;

  // ---------- [FIX 8] SENDEN mit fester Rate ----------
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;
    esp_now_send(droneAddress, (uint8_t *) &myData, sizeof(myData));
  }

  // ---------- DISPLAYS ----------
  if (now - lastDisplayUpdate > DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = now;

    // [FIX 5] Telemetrie nur verwenden, wenn sie frisch ist
    bool telemetry_fresh = (lastRecvTime != 0) && (now - lastRecvTime < TELEMETRY_TIMEOUT_MS);

    int drone_percent = -1;
    if (telemetry_fresh) {
      drone_percent = map((int)(myTelemetry.drone_voltage * 100), 1050, 1260, 0, 100);
      drone_percent = constrain(drone_percent, 0, 100);
    }

    // [FIX 15] Defekte Achse geht vor allem anderen aufs Display.
    // Eine Meldung nur auf Serial hilft nichts, wenn der Controller am
    // Akku haengt und kein USB-Kabel dran ist.
    if (stick_fault && disp1_ok) {
      display1.clearDisplay();
      display1.setTextSize(1);
      display1.setTextColor(SSD1306_WHITE);
      display1.setCursor(0, 0);
      display1.print("!! ACHSE DEFEKT !!");
      display1.setCursor(0, 16);
      display1.print(stick_fault_name);
      display1.print(" - VERKABELUNG");
      display1.display();

      if (disp2_ok) {
        display2.clearDisplay();
        display2.setTextSize(1);
        display2.setTextColor(SSD1306_WHITE);
        display2.setCursor(0, 0);
        display2.print("MODUS: [GESPERRT]");
        display2.setCursor(0, 16);
        display2.print("Armieren blockiert");
        display2.display();
      }
      return;   // Rest der Anzeige ueberspringen
    }

    if (disp1_ok) {
      display1.clearDisplay();
      display1.setTextSize(1);
      display1.setTextColor(SSD1306_WHITE);
      display1.setCursor(0, 0);
      display1.print("FUNK: ");
      display1.print(isConnected ? "VERBUNDEN" : "SUCHE...");

      display1.setCursor(0, 16);
      display1.print("C:"); display1.print(final_ctrl_percent); display1.print("% ");
      display1.print("D:");
      if (drone_percent >= 0) display1.print(drone_percent); else display1.print("--");
      display1.print("% ");

      if (!telemetry_fresh)               display1.print("...");
      else if (myTelemetry.mpu_status == 1) display1.print("CAL");
      else                                 display1.print("OK");

      // [FIX 19] Kalibrierstatus in Klartext.
      // Drift ueber 3 Grad/s -> neu kalibrieren. Die Temperatur daneben
      // sagt dir, WANN: steigt sie nicht mehr, ist die Drohne warm.
      display1.setCursor(0, 24);
      if (!telemetry_fresh) {
        display1.print("GYRO --");
      } else {
        if (fabs(myTelemetry.gyro_drift) > 3.0f) display1.print("KALIB! ");
        else                                     display1.print("GYRO ok ");
        display1.print(myTelemetry.gyro_drift, 1);
        display1.print(" ");
        display1.print((int)myTelemetry.mpu_temp);
        display1.print("C");
      }
      display1.display();
    }

    if (disp2_ok) {
      display2.clearDisplay();
      display2.setTextSize(1);
      display2.setTextColor(SSD1306_WHITE);
      display2.setCursor(0, 0);
      display2.print("MODUS: ");
      display2.print(isArmed ? "[SCHARF]" : "[ AUS ]");

      display2.setTextSize(2);
      display2.setCursor(0, 16);
      if (!isArmed) {
        display2.print("AUS");
      } else {
        int percent = map((int)currentThrottle, THROTTLE_MIN, THROTTLE_MAX, 0, 100);
        percent = constrain(percent, 0, 100);
        display2.print(percent); display2.print(" %");
      }
      display2.display();
    }
  }

  delay(5);
}
