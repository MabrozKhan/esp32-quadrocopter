#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include "QuickPID.h"

// =====================================================================
// V10.6 - Korrekturen gegenueber V10.5
//
// [FIX A] KOMPLEMENTAERFILTER: alpha war 0.98 bei dt=0.001 -> Zeitkonstante
//         49 ms. Der Lagewinkel war damit fast reiner Accel-Wert und hat
//         jede Vibration und jede Querbeschleunigung als Schraeglage
//         geglaubt. Das ist die wahrscheinlichste Quelle des Dauerfehlers,
//         den der I-Term in V10.5 aufintegriert hat. Jetzt ueber COMP_TAU
//         eingestellt (1.0 s -> alpha = 0.999).
//
// [FIX B] MOTOREN ZUERST: ledcAttach + 1000us stehen jetzt als erstes im
//         setup(). Vorher liefen die ~4-5 s MPU-Kalibrierung mit
//         floatenden Motorpins.
//
// [FIX C] RX-SNAPSHOT: Empfangsdaten werden unter Spinlock in einen
//         lokalen Snapshot kopiert. Vorher konnte der Regler auf Core 1
//         ein halb ueberschriebenes Paket lesen.
//
// [FIX D] FAILSAFE NEUTRALISIERT STICKS: bei Funkabriss gehen Roll/Pitch/
//         Yaw auf 1500. Vorher sank die Drohne mit dem letzten Stickwert.
//
// [FIX E] TELEMETRIE RAUS AUS DEM CALLBACK: esp_now_send() im
//         Empfangs-Callback ist laut Espressif nicht vorgesehen. Laeuft
//         jetzt in einem eigenen Task.
//
// [FIX F] SERIAL BLOCKIERT NICHT MEHR: groesserer TX-Puffer + Log wird
//         uebersprungen, wenn der Puffer voll ist. Vorher konnte ein
//         printf den 1-kHz-Regelkreis um bis zu 20 ms anhalten.
//
// [FIX G] I-TERM-FREIGABE MIT LATCH: einmal ueber TAKEOFF_THROTTLE
//         gewesen -> I-Term bleibt bis zum Disarm frei. Vorher war er
//         dauerhaft gedeckelt, falls das Schwebegas unter 1400 liegt.
//
// [FIX H] Reglertask auf Prioritaet 5, MPU-Verbindungstest, gefilterte
//         Akkuspannung, Winkelreset nach Kalibrierung, begrenzte
//         Soll-Raten, sanftere Landerampe.
// =====================================================================

// ---------------------------------------------------------------------
// PINS
// ---------------------------------------------------------------------
const int MOTOR_BR_PIN = 26;
const int MOTOR_FR_PIN = 25;
const int MOTOR_BL_PIN = 27;
const int MOTOR_FL_PIN = 32;

const int I2C_SDA = 21;
const int I2C_SCL = 23;

const int VBAT_PIN = 34;

// ---------------------------------------------------------------------
// EINSTELLUNGEN, DIE DU EVENTUELL ANPASSEN MUSST
// ---------------------------------------------------------------------
const int MIN_THROTTLE     = 1180;
const int MAX_THROTTLE     = 2000;
const int ARM_THROTTLE_MAX = 1220;   // nur unterhalb davon darf armiert werden
const int PID_ACTIVE_THR   = 1195;   // ab hier regelt der PID ueberhaupt
// ab hier gilt "koennte fliegen" -> I-Term wird freigegeben.
// Muss SICHER UNTER dem Schwebegas liegen, sonst ist der I-Term im Flug
// dauerhaft auf +-30 gedeckelt und die Drohne regelt Trimmfehler nie aus.
// Muss gleichzeitig UEBER dem Leerlauf (1180) liegen, damit der Windup-
// Schutz am Boden noch greift.
//
// Rechnung: Displayanzeige % = (Wert - 1180) / 820 * 100
//   1280 = 12 %   1400 = 27 %   1508 = 40 %   1672 = 60 %
//
// Altes Setup (2S, 134 mm Arme): Schwebegas ca. 60 % = 1672 -> 1400 war ok.
// Neues Setup (3S, 94 mm Arme): Abheben laut Messung vom 01.09. bei
// 30-40 %, also 1426 bis 1508.
//
// 1350 liegt sicher darunter (I-Term ist rechtzeitig frei), aber deutlich
// ueber dem Bereich, in dem sie noch auf den Beinen steht und kippelt.
// Die 1280 von vorher waren zu tief: der I-Term war schon frei, waehrend
// sie am Boden schaukelte, und hat das Aufschaukeln mitgefuettert.
const int TAKEOFF_THROTTLE = 1350;
// [FIX 10] Aus dem Messlog vom 01.09.: die Sticks kommen NICHT auf 1500
// zurueck, sondern liegen in Ruhe bei roll 1465, pitch 1538, yaw 1538.
// Das sind 35 bis 38 Einheiten Versatz. Mit den alten 15 kam davon ein
// Dauerbefehl durch, unter anderem eine staendige Gierrate von -1.5 Grad/s
// (im Log als "SOLL-RATE y: -0.3" bei stehendem Stick zu sehen).
// 45 deckt den gemessenen Versatz ab, kostet aber nur 9 % Stickweg.
const int RC_DEADBAND      = 45;
const int PID_LIMIT        = 400;

const uint32_t FAILSAFE_MS = 500;

// [FIX A] Zeitkonstante des Komplementaerfilters in Sekunden.
// Groesser = mehr Gyro, weniger Accel. 0.5 bis 2.0 s ist der sinnvolle Bereich.
// V10.5 entsprach 0.049 s.
const float COMP_TAU = 1.0f;

// [FIX H] Wie schnell faehrt das Gas beim Disarm runter (Einheiten pro ms).
// 1.5 = von 2000 auf 1000 in 0.67 s (V10.5, sehr hart).
// 0.4 = ca. 2.5 s, deutlich kontrollierter.
const float LANDING_RAMP = 0.4f;

// Maximale Soll-Drehrate, die der Winkelregler anfordern darf (Grad/s)
const float MAX_RATE_SP = 250.0f;

// =====================================================================
// [FIX 16] YAW-HEADING-HOLD
//
// Bisher wurde Gieren nur als Drehrate geregelt: Stick los = "dreh dich
// nicht weiter". Ein kleiner Gyro-Restfehler liess die Drohne dadurch
// langsam wegdrehen, ohne dass der Regler etwas dagegen hatte.
//
// Jetzt wird der Gierwinkel mitintegriert. Stick los = "halte die
// Richtung, die du beim Loslassen hattest". Gyro-Drift bleibt (ein paar
// Grad pro Minute, ohne Magnetometer nicht zu vermeiden), aber die
// Drohne dreht nicht mehr aktiv weg.
//
// KP_YAW_ANGLE: wie hart sie auf die Richtung zurueckzieht.
//   zu hoch -> ruckt beim Loslassen des Yaw-Sticks
//   zu tief  -> kommt nur traege zurueck
// YAW_STICK_MIN: ab welcher Stickvorgabe (Grad/s) auf Ratenmodus
//   umgeschaltet wird.
// =====================================================================
const bool  USE_YAW_HOLD      = true;
const float KP_YAW_ANGLE      = 3.0f;
const float YAW_STICK_MIN     = 2.0f;
// Wie schnell sie im Haltemodus hoechstens zurueckdreht (Grad/s).
// Ohne das wuerde sie nach einem Stoss von Hand mit den vollen 250 Grad/s
// zurueckschnappen - das ist im Flug unangenehm und unnoetig.
const float YAW_HOLD_MAX_RATE = 120.0f;

// =====================================================================
// [FIX 17] AIRMODE
//
// Der alte Mixer hat nach oben alle Motoren gemeinsam abgesenkt, nach
// unten aber nur hart auf MIN_THROTTLE geklemmt. Folge: bei wenig Gas
// liefen die Motoren, die runterregeln sollten, in den Anschlag, und
// die Drohne verlor genau beim Abheben ihre Steuerautoritaet.
//
// Airmode behaelt stattdessen die Differenz zwischen den Motoren und
// verschiebt den gemeinsamen Gaswert so, dass nichts anschlaegt. Ist die
// geforderte Differenz groesser als der ganze Stellbereich, wird sie
// gleichmaessig heruntskaliert statt einseitig abgeschnitten.
//
// Preis: bei kraeftigen Korrekturen steigt der Gesamtschub kurz an, die
// Drohne steigt dabei also leicht. Das ist der uebliche Kompromiss.
// Auf false setzen, um exakt das alte Verhalten zurueckzubekommen.
// =====================================================================
const bool USE_AIRMODE = true;

const uint32_t LOG_INTERVAL_MS = 300;

// =====================================================================
// [FIX 13] LOG-MODUS
//
//  1 = die ausfuehrliche Zeile alle 300 ms (Uebersicht, wie bisher)
//  2 = TUNING: nur 3 Zahlen, dafuer 100x pro Sekunde.
//      Gedacht fuer den Serial Plotter der Arduino-IDE
//      (Werkzeuge -> Serieller Plotter, gleiche Baudrate).
//      Damit siehst du die Schwingung als Kurve und kannst ablesen,
//      wie schnell sie ist:
//         viele schmale Zacken (ueber ca. 8 pro Sekunde) -> D zu hoch
//         breite gleichmaessige Wellen (3-8 pro Sekunde) -> P zu hoch
//         langsames Wallen (unter ca. 2 pro Sekunde)     -> I oder
//                                                          KP_ANGLE zu hoch
//  0 = aus
//
// Achse umschalten: LOG_AXIS auf 0 = Roll, 1 = Pitch.
// =====================================================================
#define LOG_MODE 1
#define LOG_AXIS 0

const uint32_t LOG_FAST_INTERVAL_MS = 10;

// ---------------------------------------------------------------------
// DATENSTRUKTUREN
// ---------------------------------------------------------------------
typedef struct struct_message {
  int  throttle;
  int  yaw;
  int  roll;
  int  pitch;
  bool isArmed;
  bool triggerCalib;
} struct_message;

// [FIX 19] Zwei Felder dazu, damit man am Controller SIEHT, wann
// kalibriert werden muss, statt es raten zu muessen.
//   gyro_drift: was das Gier-Gyro im Stillstand meldet. Muss nahe 0 sein.
//               Stand am 02.09. bei -17.3 - genau das soll sichtbar werden.
//   mpu_temp:   Chiptemperatur. Steigt sie nicht mehr, ist die Drohne warm
//               und der Moment zum Kalibrieren ist da.
typedef struct struct_telemetry {
  float drone_voltage;
  int   mpu_status;
  float gyro_drift;
  float mpu_temp;
} struct_telemetry;

// [FIX C] Gemeinsame RX-Daten, nur unter rxMux anfassen.
static portMUX_TYPE   rxMux    = portMUX_INITIALIZER_UNLOCKED;
static struct_message rxData   = { 1000, 1500, 1500, 1500, false, false };
static uint32_t       rxLastMs = 0;

static struct_telemetry myTelemetry = { 0.0f, 0, 0.0f, 0.0f };

static uint8_t       controllerMAC[6];
static volatile bool peerPending     = false;
static volatile bool controllerKnown = false;

volatile float battery_voltage = 0.0f;

MPU6050 mpu;
SemaphoreHandle_t i2cMutex;

float gyro_cal_roll = 0, gyro_cal_pitch = 0, gyro_cal_yaw = 0;
float angle_pitch = 0, angle_roll = 0;

float TARA_PITCH = 0.0;
float TARA_ROLL  = 0.0;
float TRIM_YAW   = 0.0;

// =====================================================================
// [FIX 12] Dauertrimm in Grad, gegen eine staendige Schraeglage.
//
// ERST kalibrieren auf einer wirklich waagerechten Flaeche probieren -
// die Kalibrierung legt fest, welche Lage die Drohne fuer "eben" haelt.
// Stand sie beim Kalibrieren schief, haelt sie genau diese Schieflage.
// Bleibt danach noch ein Rest, hier wegtrimmen.
//
// Vorzeichen ist von der Einbaulage abhaengig: mit 2.0 anfangen, wird es
// schlimmer, dann -2.0 nehmen. In 0.5er-Schritten nachziehen.
// =====================================================================
const float TRIM_PITCH_DEG = 0.0f;
const float TRIM_ROLL_DEG  = 0.0f;

const float dt = 0.001f;

// ---------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------
const float KP_ANGLE = 4.0f;

float rate_sp_roll  = 0, rate_in_roll  = 0, pwm_out_roll  = 0;
float rate_sp_pitch = 0, rate_in_pitch = 0, pwm_out_pitch = 0;
float rate_sp_yaw   = 0, rate_in_yaw   = 0, pwm_out_yaw   = 0;

// =====================================================================
// Diese Werte sind bereits fuer den neuen Aufbau (3S, 94 mm Arme)
// angepasst, ca. 30 % unter den Werten des alten Aufbaus (2S, 134 mm).
// Hintergrund der Reduktion: mehr Spannung = mehr Schubdifferenz pro
// Mikrosekunde Stellsignal, kuerzere Arme = kleineres Traegheitsmoment.
// Beides zusammen ergibt bei gleicher Reglerausgabe mehr Winkel-
// beschleunigung.
//
// Zum Feintuning im Flug:
//   schnelles Zittern / Summen  -> Kd zuerst runter, dann Kp
//   langsames Aufschaukeln      -> Kp runter
//   kippt langsam weg, kommt nicht zurueck -> Ki hoch
//   Drift in eine Richtung, sonst stabil   -> nicht der Regler,
//     sondern die Accel-Tara: eben hinstellen und neu kalibrieren
// =====================================================================

float Kp_R = 0.35, Ki_R = 0.3, Kd_R = 0.0007;
float Kp_P = 0.4,  Ki_P = 0.3, Kd_P = 0.0007;
float Kp_Y = 0.6,  Ki_Y = 0.3, Kd_Y = 0.0;

QuickPID pidRoll(&rate_in_roll, &pwm_out_roll, &rate_sp_roll, Kp_R, Ki_R, Kd_R,
                 QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
                 QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);

QuickPID pidPitch(&rate_in_pitch, &pwm_out_pitch, &rate_sp_pitch, Kp_P, Ki_P, Kd_P,
                  QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
                  QuickPID::iAwMode::iAwCondition, QuickPID::Action::reverse);

QuickPID pidYaw(&rate_in_yaw, &pwm_out_yaw, &rate_sp_yaw, Kp_Y, Ki_Y, Kd_Y,
                QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
                QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;

void IRAM_ATTR onTimer() {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

// ---------------------------------------------------------------------
// MOTOREN
// ---------------------------------------------------------------------
void writeMotor(int pin, int us) {
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  uint32_t duty = ((uint32_t)us * 8192UL) / 2500UL;   // 400 Hz -> 2500 us Periode, 13 Bit
  ledcWrite(pin, duty);
}

void allMotors(int us) {
  writeMotor(MOTOR_FL_PIN, us);
  writeMotor(MOTOR_FR_PIN, us);
  writeMotor(MOTOR_BL_PIN, us);
  writeMotor(MOTOR_BR_PIN, us);
}

// [FIX B] Muss VOR allem laufen, was Zeit kostet.
void initMotorsSafe() {
  ledcAttach(MOTOR_FL_PIN, 400, 13);
  ledcAttach(MOTOR_FR_PIN, 400, 13);
  ledcAttach(MOTOR_BL_PIN, 400, 13);
  ledcAttach(MOTOR_BR_PIN, 400, 13);
  allMotors(1000);
}

// ---------------------------------------------------------------------
// KALIBRIERUNG
// ---------------------------------------------------------------------
void safeCalibrateMPU() {
  myTelemetry.mpu_status = 1;   // Telemetrie-Task verschickt das innerhalb 100 ms

  float acc_roll_sum  = 0;
  float acc_pitch_sum = 0;
  gyro_cal_roll = 0; gyro_cal_pitch = 0; gyro_cal_yaw = 0;

  Serial.println("Kalibriere Gyroskop und Accelerometer (bitte NICHT bewegen!)...");

  for (int i = 0; i < 2000; i++) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      xSemaphoreGive(i2cMutex);
    }
    gyro_cal_roll += gx; gyro_cal_pitch += gy; gyro_cal_yaw += gz;
    float accel_x = ax / 4096.0f, accel_y = ay / 4096.0f, accel_z = az / 4096.0f;
    acc_roll_sum  += (atan2(accel_y, accel_z) * 180.0 / PI);
    acc_pitch_sum += (atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI);
    delay(2);
  }

  gyro_cal_roll /= 2000.0f; gyro_cal_pitch /= 2000.0f; gyro_cal_yaw /= 2000.0f;
  TARA_ROLL  = -(acc_roll_sum  / 2000.0f);
  TARA_PITCH = -(acc_pitch_sum / 2000.0f);

  // [FIX H] Filter neu starten, sonst schleppt er den alten Winkel mit.
  angle_roll  = 0.0f;
  angle_pitch = 0.0f;

  Serial.print("Auto-TARA ROLL: ");  Serial.println(TARA_ROLL);
  Serial.print("Auto-TARA PITCH: "); Serial.println(TARA_PITCH);

  myTelemetry.mpu_status = 0;
}

// ---------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  const uint8_t *src = info->src_addr;
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  const uint8_t *src = mac;
#endif
  // [FIX C] Nur vollstaendige Pakete uebernehmen.
  if (len != (int)sizeof(struct_message)) return;

  struct_message tmp;
  memcpy(&tmp, incomingData, sizeof(tmp));

  portENTER_CRITICAL(&rxMux);
  rxData   = tmp;
  rxLastMs = millis();
  portEXIT_CRITICAL(&rxMux);

  // [FIX E] Peer-Anmeldung und Senden passieren im Telemetrie-Task,
  // nicht hier im Callback.
  if (!controllerKnown && !peerPending) {
    memcpy(controllerMAC, src, 6);
    peerPending = true;
  }
}

void telemetryTask(void *pvParameters) {
  for (;;) {
    if (peerPending && !controllerKnown) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, controllerMAC, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) == ESP_OK) controllerKnown = true;
      peerPending = false;
    }

    if (controllerKnown) {
      // [FIX 19] Chiptemperatur nur etwa einmal pro Sekunde holen, das ist
      // ein zusaetzlicher I2C-Zugriff. Mit Timeout statt portMAX_DELAY,
      // damit dieser Task nie am Mutex haengen bleibt.
      static uint8_t tempDivider = 0;
      if (++tempDivider >= 10) {
        tempDivider = 0;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          int16_t traw = mpu.getTemperature();
          xSemaphoreGive(i2cMutex);
          myTelemetry.mpu_temp = traw / 340.0f + 36.53f;   // Formel aus dem Datenblatt
        }
      }

      myTelemetry.drone_voltage = battery_voltage;
      esp_now_send(controllerMAC, (uint8_t *) &myTelemetry, sizeof(myTelemetry));
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------------------
// HILFSFUNKTIONEN
// ---------------------------------------------------------------------
float ApplyDeadband(int rc_value, int center, int deadband) {
  if (rc_value > center + deadband) return rc_value - deadband;
  if (rc_value < center - deadband) return rc_value + deadband;
  return center;
}

void batteryMonitorTask(void *pvParameters) {
  // [FIX H] Erstwert setzen, damit die VBAT-Kompensation nicht mit 0 startet.
  battery_voltage = (analogRead(VBAT_PIN) / 4095.0f) * 3.3f * 4.2049f;
  for (;;) {
    float raw = (analogRead(VBAT_PIN) / 4095.0f) * 3.3f * 4.2049f;
    battery_voltage = battery_voltage * 0.8f + raw * 0.2f;   // EMA gegen ADC-Rauschen
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------------------
// REGELKREIS
// ---------------------------------------------------------------------
void flightControlTask(void *pvParameters) {
  static float filtered_rate_roll = 0, filtered_rate_pitch = 0, filtered_rate_yaw = 0;
  const float pt1_alpha = 0.3f;

  static bool  is_flying      = false;
  static float soft_throttle  = 1000.0f;
  static bool  pid_was_active = false;
  static bool  has_taken_off  = false;

  // [FIX 16] Aufintegrierter Gierwinkel und die zu haltende Richtung, in Grad.
  static float yaw_heading     = 0.0f;
  static float yaw_hold_target = 0.0f;

  static int  runaway_counter = 0;
  static bool emergency_stop  = false;
  const float RUNAWAY_OUTPUT_THRESHOLD = 350.0f;

  const float GROUND_ITERM_LIMIT = 30.0f;

  // [FIX A] alpha aus der Zeitkonstante statt hart 0.98
  const float comp_alpha = COMP_TAU / (COMP_TAU + dt);

  static uint32_t lastPrintTime = 0;
  static uint32_t lastArmWarn   = 0;

  for (;;) {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) != pdTRUE) continue;

    // ---------- [FIX C] Konsistenter Snapshot der Funkdaten ----------
    struct_message cmd;
    uint32_t lastRx;
    portENTER_CRITICAL(&rxMux);
    cmd    = rxData;
    lastRx = rxLastMs;
    portEXIT_CRITICAL(&rxMux);

    bool armed_from_rx = cmd.isArmed;
    bool link_ok = (millis() - lastRx) <= FAILSAFE_MS;

    // ---------- [FIX D] Failsafe: Sticks mit neutralisieren ----------
    if (!link_ok) {
      cmd.isArmed      = false;
      cmd.triggerCalib = false;
      cmd.roll  = 1500;
      cmd.pitch = 1500;
      cmd.yaw   = 1500;
      armed_from_rx = false;
    }

    // Notstopp nur loesen, wenn der Pilot wirklich disarmt hat.
    if (!armed_from_rx) {
      emergency_stop  = false;
      runaway_counter = 0;
      has_taken_off   = false;
    }

    // ---------- Kalibrierung ----------
    if (cmd.triggerCalib && !cmd.isArmed && cmd.throttle < 1050 && !is_flying) {
      allMotors(1000);
      safeCalibrateMPU();
      continue;   // dieser Zyklus ist verbrannt, sauber neu einsteigen
    }

    // ---------- Arming / Gasrampe ----------
    if (cmd.isArmed) {
      if (!is_flying) {
        if (cmd.throttle < ARM_THROTTLE_MAX) {
          is_flying     = true;
          soft_throttle = cmd.throttle;
        } else if (millis() - lastArmWarn > 1000) {
          lastArmWarn = millis();
          Serial.printf("ARM abgelehnt: Gas zu hoch (%d, noetig < %d)\n",
                        cmd.throttle, ARM_THROTTLE_MAX);
        }
      } else {
        soft_throttle = cmd.throttle;
      }
    } else {
      if (is_flying) {
        soft_throttle -= LANDING_RAMP;
        if (soft_throttle <= 1000.0f) {
          soft_throttle = 1000.0f;
          is_flying = false;
        }
      } else {
        soft_throttle = 1000.0f;
      }
    }

    // ---------- Sensorik ----------
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      xSemaphoreGive(i2cMutex);
    }

    float gyro_rate_roll  = (gx - gyro_cal_roll)  / 65.5f;
    float gyro_rate_pitch = (gy - gyro_cal_pitch) / 65.5f;
    float gyro_rate_yaw   = (gz - gyro_cal_yaw)   / 65.5f;

    float accel_x = ax / 4096.0f, accel_y = ay / 4096.0f, accel_z = az / 4096.0f;

    float accel_angle_roll  = (atan2(accel_y, accel_z) * 180.0 / PI) + TARA_ROLL;
    float accel_angle_pitch = (atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI) + TARA_PITCH;

    // [FIX A] Der eigentliche Kern der Korrektur
    angle_roll  = comp_alpha * (angle_roll  + gyro_rate_roll  * dt) + (1.0f - comp_alpha) * accel_angle_roll;
    angle_pitch = comp_alpha * (angle_pitch + gyro_rate_pitch * dt) + (1.0f - comp_alpha) * accel_angle_pitch;

    // [FIX 16] Gierwinkel aufintegrieren. Ohne Magnetometer gibt es keine
    // Stuetzung, das driftet also langsam - fuer "Richtung halten" ueber
    // eine Flugdauer reicht es aber deutlich.
    yaw_heading += gyro_rate_yaw * dt;

    // [FIX 19] Gier-Drift schaetzen, solange die Drohne steht. Was das
    // Gyro dann meldet, ist per Definition Fehler - genau dieser Wert
    // muss nach einer guten Kalibrierung nahe 0 liegen.
    if (!is_flying) {
      static float drift_est = 0.0f;
      drift_est = drift_est * 0.999f + gyro_rate_yaw * 0.001f;
      myTelemetry.gyro_drift = drift_est;
    }

    filtered_rate_roll  = (pt1_alpha * gyro_rate_roll)  + ((1.0f - pt1_alpha) * filtered_rate_roll);
    filtered_rate_pitch = (pt1_alpha * gyro_rate_pitch) + ((1.0f - pt1_alpha) * filtered_rate_pitch);
    filtered_rate_yaw   = (pt1_alpha * gyro_rate_yaw)   + ((1.0f - pt1_alpha) * filtered_rate_yaw);

    rate_in_roll  = filtered_rate_roll;
    rate_in_pitch = filtered_rate_pitch;
    rate_in_yaw   = filtered_rate_yaw;

    // ---------- Stickaufbereitung ----------
    float rc_roll_clean  = ApplyDeadband(constrain(cmd.roll,  1000, 2000), 1500, RC_DEADBAND);
    float rc_pitch_clean = ApplyDeadband(constrain(cmd.pitch, 1000, 2000), 1500, RC_DEADBAND);
    float rc_yaw_clean   = ApplyDeadband(constrain(cmd.yaw,   1000, 2000), 1500, RC_DEADBAND);

    float norm_roll  = (rc_roll_clean  - 1500) / 500.0f;
    float norm_pitch = (rc_pitch_clean - 1500) / 500.0f;
    float norm_yaw   = (rc_yaw_clean   - 1500) / 500.0f;

    const float expo_faktor = 0.6f;
    float expo_roll  = (norm_roll  * norm_roll  * norm_roll)  * expo_faktor + norm_roll  * (1.0f - expo_faktor);
    float expo_pitch = (norm_pitch * norm_pitch * norm_pitch) * expo_faktor + norm_pitch * (1.0f - expo_faktor);
    float expo_yaw   = (norm_yaw   * norm_yaw   * norm_yaw)   * expo_faktor + norm_yaw   * (1.0f - expo_faktor);

    // [FIX 12] Dauertrimm draufaddieren
    float ziel_winkel_roll  = expo_roll  *  20.0f + TRIM_ROLL_DEG;
    float ziel_winkel_pitch = expo_pitch * -20.0f + TRIM_PITCH_DEG;
    float stick_rate_yaw    = (expo_yaw  * -80.0f) + TRIM_YAW;

    // constrain() ist ein Makro und wertet sein erstes Argument mehrfach aus,
    // deshalb erst rechnen, dann begrenzen.
    float sp_r = KP_ANGLE * (ziel_winkel_roll  - angle_roll);
    float sp_p = KP_ANGLE * (ziel_winkel_pitch - angle_pitch);

    // ---------- [FIX 16] Yaw: Richtung halten statt nur Rate regeln ----------
    float sp_y;
    if (!USE_YAW_HOLD || !is_flying) {
      // Am Boden oder abgeschaltet: reiner Ratenmodus, Ziel mitfuehren
      sp_y = stick_rate_yaw;
      yaw_heading     = 0.0f;
      yaw_hold_target = 0.0f;
    } else if (fabs(stick_rate_yaw) > YAW_STICK_MIN) {
      // Pilot gibt Yaw vor -> Ratenmodus, Zielrichtung laeuft mit
      sp_y = stick_rate_yaw;
      yaw_hold_target = yaw_heading;
    } else {
      // Stick los -> auf die zuletzt gehaltene Richtung zurueckregeln
      float yaw_err = yaw_hold_target - yaw_heading;
      sp_y = constrain(KP_YAW_ANGLE * yaw_err, -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);
    }

    rate_sp_roll  = constrain(sp_r, -MAX_RATE_SP, MAX_RATE_SP);
    rate_sp_pitch = constrain(sp_p, -MAX_RATE_SP, MAX_RATE_SP);
    rate_sp_yaw   = constrain(sp_y, -MAX_RATE_SP, MAX_RATE_SP);

    // ---------- Gas / PID-Freigabe ----------
    int base_throttle = (int)soft_throttle;
    if (is_flying && cmd.isArmed && base_throttle < MIN_THROTTLE) base_throttle = MIN_THROTTLE;

    bool pid_active = (base_throttle >= PID_ACTIVE_THR);

    if (!pid_active) {
      rate_sp_roll  = rate_in_roll;
      rate_sp_pitch = rate_in_pitch;
      rate_sp_yaw   = rate_in_yaw;
    }

    // [FIX G] I-Term-Freigabe erst, wenn das Gas mindestens einmal ueber
    // TAKEOFF_THROTTLE war. Bleibt dann bis zum Disarm frei, damit ein
    // Schwebegas unterhalb 1400 den Regler nicht dauerhaft lahmlegt.
    if (base_throttle >= TAKEOFF_THROTTLE) has_taken_off = true;

    if (!has_taken_off) {
      // Vor Compute deckeln, damit schon die Ausgabe dieses Zyklus begrenzt ist.
      pidRoll.SetOutputSum (constrain(pidRoll.GetOutputSum(),  -GROUND_ITERM_LIMIT, GROUND_ITERM_LIMIT));
      pidPitch.SetOutputSum(constrain(pidPitch.GetOutputSum(), -GROUND_ITERM_LIMIT, GROUND_ITERM_LIMIT));
      pidYaw.SetOutputSum  (constrain(pidYaw.GetOutputSum(),   -GROUND_ITERM_LIMIT, GROUND_ITERM_LIMIT));
    }

    pidRoll.Compute();
    pidPitch.Compute();
    pidYaw.Compute();

    if (!pid_active) {
      pwm_out_roll = pwm_out_pitch = pwm_out_yaw = 0;
      if (pid_was_active) {
        pidRoll.SetOutputSum(0);
        pidPitch.SetOutputSum(0);
        pidYaw.SetOutputSum(0);
      }
    }
    pid_was_active = pid_active;

    // ---------- VBAT-Kompensation ----------
    if (is_flying) {
      const float max_voltage = 12.6f;
      float vbat_kompensation = 1.0f;
      if (battery_voltage > 9.0f && battery_voltage <= max_voltage) {
        vbat_kompensation = max_voltage / battery_voltage;
      }
      pwm_out_roll  *= vbat_kompensation;
      pwm_out_pitch *= vbat_kompensation;
      pwm_out_yaw   *= vbat_kompensation;
    }

    pwm_out_roll  = constrain(pwm_out_roll,  (float)-PID_LIMIT, (float)PID_LIMIT);
    pwm_out_pitch = constrain(pwm_out_pitch, (float)-PID_LIMIT, (float)PID_LIMIT);
    pwm_out_yaw   = constrain(pwm_out_yaw,   (float)-PID_LIMIT, (float)PID_LIMIT);

    // ---------- Runaway-Erkennung ----------
    bool pid_maxed = (fabs(pwm_out_roll)  > RUNAWAY_OUTPUT_THRESHOLD) ||
                     (fabs(pwm_out_pitch) > RUNAWAY_OUTPUT_THRESHOLD) ||
                     (fabs(pwm_out_yaw)   > RUNAWAY_OUTPUT_THRESHOLD);

    if (pid_active && pid_maxed) {
      if (runaway_counter < 1000) runaway_counter += 2;
    } else if (runaway_counter > 0) {
      runaway_counter--;
    }
    if (runaway_counter > 200) emergency_stop = true;

    // ---------- Mixer ----------
    int finalMotorFL, finalMotorFR, finalMotorBL, finalMotorBR;

    if (emergency_stop) {
      finalMotorFL = finalMotorFR = finalMotorBL = finalMotorBR = 1000;
      is_flying     = false;
      soft_throttle = 1000.0f;
    } else if (!is_flying) {
      finalMotorFL = finalMotorFR = finalMotorBL = finalMotorBR = 1000;
    } else {
      int p = lroundf(pwm_out_pitch);
      int r = lroundf(pwm_out_roll);
      int y = lroundf(pwm_out_yaw);

      // Reine Mischung ohne Grundgas - das ist die Differenz, die der
      // Regler zwischen den Motoren haben WILL.
      int mix_FL =  p + r + y;
      int mix_FR =  p - r - y;
      int mix_BL = -p + r - y;
      int mix_BR = -p - r + y;

      if (USE_AIRMODE && pid_active) {
        // ---------- [FIX 17] AIRMODE ----------
        int mix_max = max(max(mix_FL, mix_FR), max(mix_BL, mix_BR));
        int mix_min = min(min(mix_FL, mix_FR), min(mix_BL, mix_BR));
        int mix_range = mix_max - mix_min;
        const int allowed = MAX_THROTTLE - MIN_THROTTLE;

        // Schritt 1: Passt die geforderte Spreizung ueberhaupt in den
        // Stellbereich? Wenn nicht, alle vier gleichmaessig skalieren.
        // Damit bleiben die Verhaeltnisse zwischen den Achsen erhalten,
        // statt eine Achse einseitig abzuschneiden.
        if (mix_range > allowed && mix_range > 0) {
          float scale = (float)allowed / (float)mix_range;
          mix_FL = lroundf(mix_FL * scale);
          mix_FR = lroundf(mix_FR * scale);
          mix_BL = lroundf(mix_BL * scale);
          mix_BR = lroundf(mix_BR * scale);
          mix_max = max(max(mix_FL, mix_FR), max(mix_BL, mix_BR));
          mix_min = min(min(mix_FL, mix_FR), min(mix_BL, mix_BR));
        }

        // Schritt 2: Grundgas so verschieben, dass kein Motor anschlaegt.
        // Nach Schritt 1 passt die Spreizung garantiert, die beiden
        // Bedingungen koennen sich also nicht widersprechen.
        int base = base_throttle;
        if (base + mix_min < MIN_THROTTLE) base = MIN_THROTTLE - mix_min;
        if (base + mix_max > MAX_THROTTLE) base = MAX_THROTTLE - mix_max;

        finalMotorFL = base + mix_FL;
        finalMotorFR = base + mix_FR;
        finalMotorBL = base + mix_BL;
        finalMotorBR = base + mix_BR;
      } else {
        // ---------- Altes Verhalten: nur nach oben absenken ----------
        int m_FL_raw = base_throttle + mix_FL;
        int m_FR_raw = base_throttle + mix_FR;
        int m_BL_raw = base_throttle + mix_BL;
        int m_BR_raw = base_throttle + mix_BR;

        int max_motor = max(max(m_FL_raw, m_FR_raw), max(m_BL_raw, m_BR_raw));
        if (max_motor > MAX_THROTTLE) {
          int overflow = max_motor - MAX_THROTTLE;
          m_FL_raw -= overflow; m_FR_raw -= overflow;
          m_BL_raw -= overflow; m_BR_raw -= overflow;
        }

        finalMotorFL = m_FL_raw;
        finalMotorFR = m_FR_raw;
        finalMotorBL = m_BL_raw;
        finalMotorBR = m_BR_raw;
      }

      // Letztes Netz - nach Airmode sollte hier nichts mehr greifen.
      finalMotorFL = constrain(finalMotorFL, MIN_THROTTLE, MAX_THROTTLE);
      finalMotorFR = constrain(finalMotorFR, MIN_THROTTLE, MAX_THROTTLE);
      finalMotorBL = constrain(finalMotorBL, MIN_THROTTLE, MAX_THROTTLE);
      finalMotorBR = constrain(finalMotorBR, MIN_THROTTLE, MAX_THROTTLE);
    }

    writeMotor(MOTOR_FL_PIN, finalMotorFL);
    writeMotor(MOTOR_FR_PIN, finalMotorFR);
    writeMotor(MOTOR_BL_PIN, finalMotorBL);
    writeMotor(MOTOR_BR_PIN, finalMotorBR);

    // ---------- [FIX 13] Schnelles Tuning-Log fuer den Serial Plotter ----------
#if LOG_MODE == 2
    if (millis() - lastPrintTime >= LOG_FAST_INTERVAL_MS && Serial.availableForWrite() > 60) {
      lastPrintTime = millis();
  #if LOG_AXIS == 0
      Serial.printf("winkel:%.1f,rate:%.1f,out:%.0f\n",
                    angle_roll, rate_in_roll, pwm_out_roll);
  #else
      Serial.printf("winkel:%.1f,rate:%.1f,out:%.0f\n",
                    angle_pitch, rate_in_pitch, pwm_out_pitch);
  #endif
    }
#endif

    // ---------- [FIX F] Log, das den Regelkreis nicht anhaelt ----------
#if LOG_MODE == 1
    if (millis() - lastPrintTime >= LOG_INTERVAL_MS && Serial.availableForWrite() > 260) {
      lastPrintTime = millis();
      Serial.printf(
        "Thr:%4d | STICK r:%4d p:%4d y:%4d | WINKEL roll:%6.1f pitch:%6.1f | "
        "SOLL-RATE r:%6.1f p:%6.1f y:%6.1f | IST-RATE r:%6.1f p:%6.1f y:%6.1f | "
        "I-TERM r:%6.1f p:%6.1f y:%6.1f | OUT r:%5.0f p:%5.0f y:%5.0f | "
        "Motoren FL:%4d FR:%4d BL:%4d BR:%4d | Link:%d TO:%d Stop:%d\n",
        base_throttle,
        cmd.roll, cmd.pitch, cmd.yaw,
        angle_roll, angle_pitch,
        rate_sp_roll, rate_sp_pitch, rate_sp_yaw,
        rate_in_roll, rate_in_pitch, rate_in_yaw,
        pidRoll.GetOutputSum(), pidPitch.GetOutputSum(), pidYaw.GetOutputSum(),
        pwm_out_roll, pwm_out_pitch, pwm_out_yaw,
        finalMotorFL, finalMotorFR, finalMotorBL, finalMotorBR,
        link_ok, has_taken_off, emergency_stop
      );
    }
#endif
  }
}

// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  // [FIX B] ALLERERSTES: Motorpins definiert auf 1000 us.
  initMotorsSafe();

  // [FIX F] Groesserer TX-Puffer, damit printf spaeter nicht blockiert.
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);

  analogReadResolution(12);

  i2cMutex = xSemaphoreCreateMutex();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  mpu.initialize();

  // [FIX H] Ohne MPU darf gar nichts starten.
  if (!mpu.testConnection()) {
    Serial.println("FEHLER: MPU6050 nicht erreichbar. Motoren bleiben aus.");
    for (;;) { allMotors(1000); delay(500); }
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  safeCalibrateMPU();

  pidRoll.SetOutputLimits(-PID_LIMIT, PID_LIMIT);
  pidPitch.SetOutputLimits(-PID_LIMIT, PID_LIMIT);
  pidYaw.SetOutputLimits(-PID_LIMIT, PID_LIMIT);

  pidRoll.SetSampleTimeUs(1000);
  pidPitch.SetSampleTimeUs(1000);
  pidYaw.SetSampleTimeUs(1000);

  pidRoll.SetTunings(Kp_R, Ki_R, Kd_R);
  pidPitch.SetTunings(Kp_P, Ki_P, Kd_P);
  pidYaw.SetTunings(Kp_Y, Ki_Y, Kd_Y);

  pidRoll.SetMode(QuickPID::Control::timer);
  pidPitch.SetMode(QuickPID::Control::timer);
  pidYaw.SetMode(QuickPID::Control::timer);

  allMotors(1000);
  delay(3000);   // ESC-Startzeit

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("FEHLER: ESP-NOW Init fehlgeschlagen. Motoren bleiben aus.");
    for (;;) { allMotors(1000); delay(500); }
  }

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  #else
      esp_now_register_recv_cb(OnDataRecv);
  #endif

  timerSemaphore = xSemaphoreCreateBinary();

  // [FIX H] Regler bekommt Vorrang auf Core 1, Nebenlaeufiges auf Core 0.
  xTaskCreatePinnedToCore(flightControlTask,  "FlightTask",  10000, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(batteryMonitorTask, "BatteryTask",  2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(telemetryTask,      "TelemTask",    3072, NULL, 2, NULL, 0);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000, true, 0);
}

void loop() {
  vTaskDelete(NULL);
}
