#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32 
//neuste version
// I2C Busse für Dual-Display Setup initialisieren
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_1, -1);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_2, -1);

// Pin-Definitionen für Joysticks und Schalter
const int PIN_LEFT_YAW = 34;      
const int PIN_LEFT_THROTTLE = 35; 
const int PIN_RIGHT_ROLL = 36; // Joystick auf VP, da dieser nicht stört
const int PIN_BATTERY  = 32;   // Batterie auf dem sicheren ADC-Pin 32
const int PIN_RIGHT_PITCH = 33;   
const int PIN_BTN_ARM = 13;       
const int PIN_CALIB_SW = 4;   // Rechter Joystick Klick (Kalibrierung)

// Ziel-Adresse der Drohne (MAC-Adresse)
uint8_t droneAddress[] = {0x68, 0x09, 0x47, 0x9D, 0xFA, 0x4C};

// Datenpaket-Strukturen
typedef struct struct_cmd {
  int throttle; 
  int yaw;      
  int roll;     
  int pitch;    
  bool isArmed; 
  bool triggerCalib; 
} struct_cmd;

typedef struct struct_telemetry {
  float drone_voltage;
  int mpu_status; 
} struct_telemetry;

struct_cmd myData;
struct_telemetry myTelemetry;

// System-Variablen
bool isConnected = false;
bool isArmed = false;
bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastRecvTime = 0; 

// Batterie-Variablen
int final_ctrl_percent = -1; 
// 3.95V (Multimeter) - 3.72V (ESP Berechnung) = 0.23V Verlust durch Kabel & Akku-Alter
const float VOLTAGE_COMPENSATION = 0.23; 

// Konstanten für den virtuellen Gas-Stick
float currentThrottle = 1000.0;   
const int CENTER_THROTTLE = 1808; 
const int DEADBAND = 300;         
const float THROTTLE_SPEED = 2.0; 

// =====================================================================
// ESP-NOW CALLBACKS
// =====================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  isConnected = (status == ESP_NOW_SEND_SUCCESS);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  memcpy(&myTelemetry, incomingData, sizeof(myTelemetry));
  lastRecvTime = millis();
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_LEFT_YAW, INPUT);
  pinMode(PIN_LEFT_THROTTLE, INPUT);
  pinMode(PIN_RIGHT_ROLL, INPUT);
  pinMode(PIN_RIGHT_PITCH, INPUT);
  
  pinMode(PIN_BTN_ARM, INPUT_PULLUP);
  pinMode(PIN_CALIB_SW, INPUT_PULLUP); 

  I2C_1.begin(21, 22, 100000);
  I2C_2.begin(18, 19, 100000);
  
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // =====================================================================
  // SMART BATTERY MEASUREMENT (Mit Kompensation!)
  // =====================================================================
  long sum = 0;
  for(int i=0; i<50; i++) { 
      sum += analogRead(PIN_BATTERY);
      delay(2);
  }
  float avg_raw = sum / 50.0;
  
  // Berechnung: (Raw->Volt) + Hardware-Verlust (0.23V)
  float ruhespannung = ((avg_raw / 4095.0) * 3.3 * 2.0 * 1.08) + VOLTAGE_COMPENSATION; 
  
  // 3.5V (350) = 0%, 4.2V (420) = 100%
  final_ctrl_percent = map((int)(ruhespannung * 100), 350, 420, 0, 100);
  final_ctrl_percent = constrain(final_ctrl_percent, 0, 100);
  
  Serial.print("Kompensierte Spannung: "); Serial.println(ruhespannung);
  Serial.print("Initialer Akku: "); Serial.println(final_ctrl_percent);
  // =====================================================================

  WiFi.mode(WIFI_STA); 
  esp_now_init();
  
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
  esp_now_add_peer(&peerInfo);

  myTelemetry.mpu_status = 0;
  myTelemetry.drone_voltage = 0.0;
}

void loop() {
  // 🚀 Drohnen-Akku Berechnung jetzt für den 3S LiPo (10.5V bis 12.6V) 🚀
  int drone_percent = map(myTelemetry.drone_voltage * 100, 1050, 1260, 0, 100);
  drone_percent = constrain(drone_percent, 0, 100);

  // --- SCHALTER AUSLESEN (ARMING) ---
  int btnState = digitalRead(PIN_BTN_ARM);
  if (btnState == LOW && lastBtnState == HIGH && (millis() - lastDebounceTime) > 200) {
    isArmed = !isArmed; 
    
    if(!isArmed) {
      currentThrottle = 1000.0; 
    } else {
      currentThrottle = 1180.0; 
    }
    lastDebounceTime = millis();
  }
  lastBtnState = btnState; 

  // --- KALIBRIERUNGS-KNOPF ---
  if (!isArmed && digitalRead(PIN_CALIB_SW) == LOW) {
    myData.triggerCalib = true;
  } else {
    myData.triggerCalib = false;
  }

  // --- GAS BERECHNEN (VIRTUELLER STICK) ---
  int rawThrottle = analogRead(PIN_LEFT_THROTTLE);
  
  if (isArmed) {
    if (rawThrottle > CENTER_THROTTLE + DEADBAND) {
      currentThrottle += THROTTLE_SPEED;
    } 
    else if (rawThrottle < CENTER_THROTTLE - DEADBAND) {
      currentThrottle -= THROTTLE_SPEED;
    }
    currentThrottle = constrain(currentThrottle, 1180, 2000); 
  }

  // --- PAKET BEFÜLLEN & SENDEN ---
  myData.throttle = (int)currentThrottle;

  // 1. Rohwerte auslesen (0 bis 4095)
  int read_yaw = analogRead(PIN_LEFT_YAW);
  int read_roll = analogRead(PIN_RIGHT_ROLL);
  int read_pitch = analogRead(PIN_RIGHT_PITCH);

  // 2. Werte auf 1000-2000 skalieren und die schiefe Mitte (1808) exakt auf 1500 zwingen!
  int mapped_yaw = (read_yaw < 1808) ? map(read_yaw, 0, 1808, 1000, 1500) : map(read_yaw, 1808, 4095, 1500, 2000);
  int mapped_roll = (read_roll < 1808) ? map(read_roll, 0, 1808, 1000, 1500) : map(read_roll, 1808, 4095, 1500, 2000);
  int mapped_pitch = (read_pitch < 1808) ? map(read_pitch, 0, 1808, 1000, 1500) : map(read_pitch, 1808, 4095, 1500, 2000);

  // 3. Achsen invertieren, falls nötig (1000 wird zu 2000, 1500 bleibt 1500)
  myData.yaw = 3000 - mapped_yaw;   // Entspricht deinem alten 4095 - read
  myData.roll = mapped_roll;
  myData.pitch = 3000 - mapped_pitch; // Entspricht deinem alten 4095 - read

  myData.isArmed = isArmed;

  esp_now_send(droneAddress, (uint8_t *) &myData, sizeof(myData));

  // --- DISPLAYS AKTUALISIEREN ---
  if (millis() - lastDisplayUpdate > 100) {
    
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 0);
    display1.print("FUNK: "); display1.print(isConnected ? "VERBUNDEN" : "SUCHE...");
    
    display1.setCursor(0, 16);
    display1.print("C:"); display1.print(final_ctrl_percent); display1.print("% ");
    display1.print("D:"); display1.print(drone_percent); display1.print("% ");
    
    if (myTelemetry.mpu_status == 1) {
       display1.print("CAL");
    } else {
       display1.print("OK");
    }
    display1.display();

    display2.clearDisplay();
    display2.setTextSize(1);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor(0, 0);
    display2.print("MODUS: "); display2.print(isArmed ? "[SCHARF]" : "[ AUS ]");
    
    display2.setTextSize(2); 
    display2.setCursor(0, 16);
    if (!isArmed) {
      display2.print("AUS");
    } else {
      int percent = map((int)currentThrottle, 1180, 2000, 0, 100);
      display2.print(percent); display2.print(" %");
    }
    display2.display();

    lastDisplayUpdate = millis();
  }
  
  delay(20); 
}