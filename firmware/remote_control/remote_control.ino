// Remote control for the ESP32 quadcopter.
//
// Two KY-023 joysticks, two 128x32 OLEDs on separate I2C buses, one 18650
// cell. Sends stick values to the drone over ESP-NOW every 20 ms and shows
// the telemetry it gets back (drone battery, calibration state, gyro drift).
//
// The throttle stick springs back to centre, so it works as a "virtual"
// stick: up raises the throttle value, down lowers it, centre keeps it.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_1, -1);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_2, -1);
bool disp1_ok = false;
bool disp2_ok = false;

// ---------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------
const int PIN_LEFT_YAW      = 34;
const int PIN_LEFT_THROTTLE = 35;
const int PIN_RIGHT_ROLL    = 36;   // VP
const int PIN_BATTERY       = 32;
const int PIN_RIGHT_PITCH   = 33;
const int PIN_BTN_ARM       = 13;
const int PIN_CALIB_SW      = 4;    // right joystick button

// MAC address of the ESP32 in the drone, change this for your own board
uint8_t droneAddress[] = { 0x68, 0x09, 0x47, 0x9D, 0xFA, 0x4C };

// ---------------------------------------------------------------------
// Data structures (must match the flight controller exactly, otherwise
// the packets are dropped because of the wrong length)
// ---------------------------------------------------------------------
typedef struct struct_cmd {
  int  throttle;
  int  yaw;
  int  roll;
  int  pitch;
  bool isArmed;
  bool triggerCalib;
} struct_cmd;

typedef struct struct_telemetry {
  float drone_voltage;
  int   mpu_status;
  float gyro_drift;
  float mpu_temp;
} struct_telemetry;

struct_cmd       myData      = { 1000, 1500, 1500, 1500, false, false };
struct_telemetry myTelemetry = { 0.0f, 0 };

// ---------------------------------------------------------------------
// State and timing
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
// Remote battery
// ---------------------------------------------------------------------
int final_ctrl_percent = -1;
// measured 3.95 V with a multimeter, ESP calculated 3.72 V
const float VOLTAGE_COMPENSATION = 0.23f;

// ---------------------------------------------------------------------
// Virtual throttle stick
// ---------------------------------------------------------------------
const int   THROTTLE_MIN = 1180;   // same as MIN_THROTTLE on the drone
const int   THROTTLE_MAX = 2000;

// Dead zone around the throttle stick centre in raw ADC counts. Large on
// purpose: the stick overshoots a bit when released.
const int THROTTLE_DEADBAND = 450;

// After an active deflection, the opposite direction is ignored for this
// long. Otherwise the overshoot on release counted as a short "down" and the
// throttle dropped by 1-2 %.
const unsigned long RELEASE_LOCKOUT_MS = 200;

const bool USE_MEASURED_CENTER = true;   // false = fixed centre of 1808

// Throttle ramp in us per second. 200 takes about 1.3 s from idle to hover,
// with 100 the drone sat in ground effect too long and started rocking.
const float THROTTLE_RATE_PER_S = 200.0f;

float currentThrottle = 1000.0f;

// Axis inversion, change it here and not in the drone's mixer
const bool INVERT_ROLL  = false;
const bool INVERT_PITCH = false;
const bool INVERT_YAW   = true;

// Stick centres, measured at startup
const int CENTER_FALLBACK = 1808;
int center_yaw   = CENTER_FALLBACK;
int center_roll  = CENTER_FALLBACK;
int center_pitch = CENTER_FALLBACK;
int center_thr   = CENTER_FALLBACK;

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

// The first reading after switching ADC channel can be wrong on the ESP32
// (GPIO36/39), so it is thrown away and the rest is averaged.
// No default argument on purpose: the Arduino IDE copies it into the
// generated prototype and the build fails.
int readAdc(int pin, int samples) {
  analogRead(pin);
  long sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return (int)(sum / samples);
}

// Map raw ADC to 1000..2000 with the measured centre at exactly 1500
int mapStick(int raw, int center) {
  center = constrain(center, 500, 3500);   // keeps map() away from a division by 0
  int v;
  if (raw < center) v = map(raw, 0, center, 1000, 1500);
  else              v = map(raw, center, 4095, 1500, 2000);
  return constrain(v, 1000, 2000);
}

// A stick that is far off centre at startup is treated as broken and blocks
// arming. This found a broken potentiometer wire on the roll axis (it read
// about 15 all the time and the drone got a constant -16 deg command).
bool stick_fault = false;
char stick_fault_name[10] = "";

int measureCenter(int pin, const char *name) {
  int v = readAdc(pin, 32);
  if (v < 1400 || v > 2400) {
    Serial.printf("ERROR axis %s (pin %d): rest value %d, expected 1400..2400.\n", name, pin, v);
    Serial.println("       Close to 0 or 4095 usually means a broken pot wire.");
    Serial.println("       Arming is blocked until this is fixed and the remote restarted.");
    stick_fault = true;
    strncpy(stick_fault_name, name, sizeof(stick_fault_name) - 1);
    stick_fault_name[sizeof(stick_fault_name) - 1] = '\0';
    return CENTER_FALLBACK;
  }
  return v;
}

// ---------------------------------------------------------------------
// ESP-NOW callbacks
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
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

  disp1_ok = display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  disp2_ok = display2.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!disp1_ok) Serial.println("WARNING: display 1 not found.");
  if (!disp2_ok) Serial.println("WARNING: display 2 not found.");

  // measure stick centres, do not touch the sticks while powering on
  delay(200);
  if (USE_MEASURED_CENTER) {
    center_yaw   = measureCenter(PIN_LEFT_YAW,      "YAW");
    center_thr   = measureCenter(PIN_LEFT_THROTTLE, "THR");
    center_roll  = measureCenter(PIN_RIGHT_ROLL,    "ROLL");
    center_pitch = measureCenter(PIN_RIGHT_PITCH,   "PITCH");
  }
  Serial.printf("Centres: yaw=%d thr=%d roll=%d pitch=%d (fallback: %d)\n",
                center_yaw, center_thr, center_roll, center_pitch, CENTER_FALLBACK);
  Serial.printf("Throttle window: down below %d, up above %d\n",
                center_thr - THROTTLE_DEADBAND, center_thr + THROTTLE_DEADBAND);

  // battery, measured once at startup without load (10k / 10k divider)
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(PIN_BATTERY);
    delay(2);
  }
  float avg_raw = sum / 50.0f;
  float restVoltage = ((avg_raw / 4095.0f) * 3.3f * 2.0f * 1.08f) + VOLTAGE_COMPENSATION;

  // 3.5 V = 0 %, 4.2 V = 100 %
  final_ctrl_percent = map((int)(restVoltage * 100), 350, 420, 0, 100);
  final_ctrl_percent = constrain(final_ctrl_percent, 0, 100);

  Serial.print("Battery voltage: ");  Serial.println(restVoltage);
  Serial.print("Battery percent: ");  Serial.println(final_ctrl_percent);

  // radio
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed.");
    if (disp1_ok) {
      display1.clearDisplay();
      display1.setTextSize(1);
      display1.setTextColor(SSD1306_WHITE);
      display1.setCursor(0, 0);
      display1.print("ESP-NOW ERROR");
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
    Serial.println("ERROR: could not add peer.");
  }

  lastRampMicros = micros();
}

// ---------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // ---------- read sticks ----------
  int rawThrottle = readAdc(PIN_LEFT_THROTTLE, 4);
  int read_yaw    = readAdc(PIN_LEFT_YAW, 4);
  int read_roll   = readAdc(PIN_RIGHT_ROLL, 4);
  int read_pitch  = readAdc(PIN_RIGHT_PITCH, 4);

  bool throttleStickCentered = (rawThrottle <= center_thr + THROTTLE_DEADBAND) &&
                               (rawThrottle >= center_thr - THROTTLE_DEADBAND);

  // all four axes have to be at rest to arm
  const int STICK_ARM_TOLERANCE = 400;   // raw counts
  bool allSticksCentered = throttleStickCentered &&
      (abs(read_roll  - center_roll)  < STICK_ARM_TOLERANCE) &&
      (abs(read_pitch - center_pitch) < STICK_ARM_TOLERANCE) &&
      (abs(read_yaw   - center_yaw)   < STICK_ARM_TOLERANCE);

  // ---------- arm button ----------
  int btnState = digitalRead(PIN_BTN_ARM);
  if (btnState == LOW && lastBtnState == HIGH && (now - lastDebounceTime) > 200) {
    if (isArmed) {
      // disarm is always allowed
      isArmed = false;
      currentThrottle = 1000.0f;
      lastDebounceTime = now;
    } else if (stick_fault) {
      Serial.printf("Arming refused: axis %s faulty, check the wiring.\n", stick_fault_name);
      lastDebounceTime = now;
    } else if (!allSticksCentered) {
      Serial.printf("Arming refused: stick not centred (thr:%d roll:%d pitch:%d yaw:%d).\n",
                    rawThrottle, read_roll, read_pitch, read_yaw);
      lastDebounceTime = now;
    } else {
      isArmed = true;
      currentThrottle = (float)THROTTLE_MIN;
      lastDebounceTime = now;
    }
  }
  lastBtnState = btnState;

  // ---------- calibration request ----------
  myData.triggerCalib = (!isArmed && digitalRead(PIN_CALIB_SW) == LOW);

  // ---------- virtual throttle, time based ----------
  unsigned long nowUs = micros();
  float dt = (nowUs - lastRampMicros) / 1000000.0f;
  lastRampMicros = nowUs;
  // a display update blocks the loop for about 25 ms, so dt is capped
  if (dt > 0.05f) dt = 0.05f;

  int stickDir = 0;
  if (rawThrottle > center_thr + THROTTLE_DEADBAND)      stickDir = +1;
  else if (rawThrottle < center_thr - THROTTLE_DEADBAND) stickDir = -1;

  // ignore the overshoot right after releasing the stick
  static int           lastActiveDir = 0;
  static unsigned long lastActiveMs  = 0;

  if (stickDir != 0) {
    if (stickDir == -lastActiveDir && (now - lastActiveMs) < RELEASE_LOCKOUT_MS) {
      stickDir = 0;
    } else {
      lastActiveDir = stickDir;
      lastActiveMs  = now;
    }
  }

  if (isArmed) {
    float step = THROTTLE_RATE_PER_S * dt;
    if (stickDir > 0)      currentThrottle += step;
    else if (stickDir < 0) currentThrottle -= step;
    currentThrottle = constrain(currentThrottle, (float)THROTTLE_MIN, (float)THROTTLE_MAX);
  } else {
    currentThrottle = 1000.0f;
    lastActiveDir = 0;
  }

  // Raw value / centre for every axis. An axis far from its centre at rest,
  // or one that barely changes when moved, points to a hardware fault.
#define DEBUG_THROTTLE 1
#if DEBUG_THROTTLE
  static unsigned long lastThrDbg = 0;
  if (now - lastThrDbg > 500) {
    lastThrDbg = now;
    Serial.printf("thr:%4d/%4d  roll:%4d/%4d  pitch:%4d/%4d  yaw:%4d/%4d | Margin:%4d Dir:%+d Thr:%4d\n",
                  rawThrottle, center_thr,
                  read_roll,   center_roll,
                  read_pitch,  center_pitch,
                  read_yaw,    center_yaw,
                  THROTTLE_DEADBAND - abs(rawThrottle - center_thr),
                  stickDir, (int)currentThrottle);
  }
#endif

  // ---------- build packet ----------
  myData.throttle = (int)currentThrottle;

  int mapped_yaw   = mapStick(read_yaw,   center_yaw);
  int mapped_roll  = mapStick(read_roll,  center_roll);
  int mapped_pitch = mapStick(read_pitch, center_pitch);

  myData.roll  = INVERT_ROLL  ? (3000 - mapped_roll)  : mapped_roll;
  myData.pitch = INVERT_PITCH ? (3000 - mapped_pitch) : mapped_pitch;
  myData.yaw   = INVERT_YAW   ? (3000 - mapped_yaw)   : mapped_yaw;

  myData.isArmed = isArmed;

  // ---------- send at a fixed rate ----------
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;
    esp_now_send(droneAddress, (uint8_t *) &myData, sizeof(myData));
  }

  // ---------- displays ----------
  if (now - lastDisplayUpdate > DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = now;

    bool telemetry_fresh = (lastRecvTime != 0) && (now - lastRecvTime < TELEMETRY_TIMEOUT_MS);

    int drone_percent = -1;
    if (telemetry_fresh) {
      drone_percent = map((int)(myTelemetry.drone_voltage * 100), 1050, 1260, 0, 100);
      drone_percent = constrain(drone_percent, 0, 100);
    }

    // a stick fault has to be visible without a USB cable
    if (stick_fault && disp1_ok) {
      display1.clearDisplay();
      display1.setTextSize(1);
      display1.setTextColor(SSD1306_WHITE);
      display1.setCursor(0, 0);
      display1.print("!! AXIS FAULT !!");
      display1.setCursor(0, 16);
      display1.print(stick_fault_name);
      display1.print(" - CHECK WIRING");
      display1.display();

      if (disp2_ok) {
        display2.clearDisplay();
        display2.setTextSize(1);
        display2.setTextColor(SSD1306_WHITE);
        display2.setCursor(0, 0);
        display2.print("MODE: [LOCKED]");
        display2.setCursor(0, 16);
        display2.print("Arming blocked");
        display2.display();
      }
      return;
    }

    if (disp1_ok) {
      display1.clearDisplay();
      display1.setTextSize(1);
      display1.setTextColor(SSD1306_WHITE);
      display1.setCursor(0, 0);
      display1.print("LINK: ");
      display1.print(isConnected ? "CONNECTED" : "SEARCHING");

      display1.setCursor(0, 16);
      display1.print("C:"); display1.print(final_ctrl_percent); display1.print("% ");
      display1.print("D:");
      if (drone_percent >= 0) display1.print(drone_percent); else display1.print("--");
      display1.print("% ");

      if (!telemetry_fresh)               display1.print("...");
      else if (myTelemetry.mpu_status == 1) display1.print("CAL");
      else                                 display1.print("OK");

      // gyro drift above 3 deg/s means recalibrate, once the temperature
      // has stopped rising
      display1.setCursor(0, 24);
      if (!telemetry_fresh) {
        display1.print("GYRO --");
      } else {
        if (fabs(myTelemetry.gyro_drift) > 3.0f) display1.print("RECAL! ");
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
      display2.print("MODE: ");
      display2.print(isArmed ? "[ARMED]" : "[ OFF ]");

      display2.setTextSize(2);
      display2.setCursor(0, 16);
      if (!isArmed) {
        display2.print("OFF");
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
