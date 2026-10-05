#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h> 
#include <MPU6050.h> 
#include "QuickPID.h"

// =====================================================================
// V10.5 - AUF BASIS DER ECHTEN MESSDATEN AUS DEM SERIAL-LOG
//
// BEFUND: Die Motorwerte im Log sind mathematisch korrekt aus den PID-
// Ausgaben berechnet (Mixer nachgerechnet, stimmt exakt). Das Problem ist
// die PID-Ausgabe selbst: Pitch und Yaw wachsen ueber Sekunden STETIG an
// (269->271->290->292->311, bzw. -321->-326->-350->-371). Das ist kein
// Spike, sondern die klassische Signatur von I-TERM-WINDUP.
//
// URSACHE: Die Drohne steht am Boden und kann den Regelfehler physikalisch
// nicht ausregeln. Der I-Term integriert ihn trotzdem weiter, bis er das
// +-400-Limit erreicht. Der bisherige "Boden-Schutz" griff nur unterhalb
// Throttle 1195 - im Log steht aber Thr:1220, also genau darueber. Roll
// blieb sauber (3-6), weil dort kein Dauerfehler anliegt; Pitch und Yaw
// haben einen (Ursache noch zu klaeren: Stick-Zentrierung oder angle_pitch).
//
// FIXES:
// 1) I-Term wird hart begrenzt, solange base_throttle unter TAKEOFF_THROTTLE
//    liegt - der Regler kann am Boden also gar nicht mehr hochlaufen.
// 2) VBAT-Kompensation wird nach der Multiplikation neu geclampt. Vorher
//    konnte sie das +-400-Limit aushebeln (im Log: Y:-479!).
// 3) Runaway-Erkennung mit Zaehler statt hartem Reset - ein einzelner
//    Zyklus unter der Schwelle setzt sie nicht mehr komplett zurueck
//    (deshalb stand im Log durchgehend Notstopp:0 trotz Werten ueber 350).
// 4) Erweitertes Debug-Log: rohe Stickwerte, Winkel, Soll-/Ist-Raten und
//    die I-Terme einzeln - damit sichtbar wird, WOHER der Dauerfehler kommt.
// =====================================================================

const int MOTOR_BR_PIN = 26; 
const int MOTOR_FR_PIN = 25; 
const int MOTOR_BL_PIN = 27; 
const int MOTOR_FL_PIN = 32; 

const int I2C_SDA = 21;
const int I2C_SCL = 23;

const int VBAT_PIN = 34; 
volatile float battery_voltage = 0.0; 

MPU6050 mpu;
SemaphoreHandle_t i2cMutex;

float gyro_cal_roll = 0, gyro_cal_pitch = 0, gyro_cal_yaw = 0;
float angle_pitch = 0, angle_roll = 0, angle_yaw = 0;

float TARA_PITCH = 0.0; 
float TARA_ROLL  = 0.0;  
float TRIM_YAW   = 0.0; 

const float dt = 0.001; 

typedef struct struct_message {
  int throttle; 
  int yaw;      
  int roll;     
  int pitch;    
  bool isArmed; 
  bool triggerCalib; 
} struct_message;

typedef struct struct_telemetry {
  float drone_voltage;
  int mpu_status; 
} struct_telemetry;

volatile struct_message myData; 
volatile struct_telemetry myTelemetry;

unsigned long lastRecvTime = 0; 
uint8_t controllerMAC[6]; 
bool controllerKnown = false;

const int MIN_THROTTLE = 1180;     
const int MAX_THROTTLE = 2000; 
const int RC_DEADBAND = 15; 

const int PID_LIMIT = 400;

void writeMotor(int pin, int us) {
  uint32_t duty = (us * 8192) / 2500;
  ledcWrite(pin, duty);
}

const float KP_ANGLE = 4.0; 

float rate_sp_roll = 0, rate_in_roll = 0, pwm_out_roll = 0;
float rate_sp_pitch = 0, rate_in_pitch = 0, pwm_out_pitch = 0;
float rate_sp_yaw = 0, rate_in_yaw = 0, pwm_out_yaw = 0;

float Kp_R = 0.35, Ki_R = 0.3, Kd_R = 0.0007; 
float Kp_P = 0.4,  Ki_P = 0.3, Kd_P = 0.0007; 
float Kp_Y = 0.6,  Ki_Y = 0.3, Kd_Y = 0.0; 

QuickPID pidRoll(&rate_in_roll, &pwm_out_roll, &rate_sp_roll, Kp_R, Ki_R, Kd_R,
                 QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);

QuickPID pidPitch(&rate_in_pitch, &pwm_out_pitch, &rate_sp_pitch, Kp_P, Ki_P, Kd_P,
                  QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::reverse);

QuickPID pidYaw(&rate_in_yaw, &pwm_out_yaw, &rate_sp_yaw, Kp_Y, Ki_Y, Kd_Y,
                QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;

void IRAM_ATTR onTimer() {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void safeCalibrateMPU() {
  myTelemetry.mpu_status = 1; 
  if (controllerKnown) esp_now_send(controllerMAC, (uint8_t *) &myTelemetry, sizeof(myTelemetry));
  
  float acc_roll_sum = 0;
  float acc_pitch_sum = 0;
  gyro_cal_roll = 0; gyro_cal_pitch = 0; gyro_cal_yaw = 0;

  Serial.println("Kalibriere Gyroskop & Accelerometer (bitte NICHT bewegen!)...");
  
  for (int i = 0; i < 2000; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyro_cal_roll += gx; gyro_cal_pitch += gy; gyro_cal_yaw += gz;
    float accel_y = ay / 4096.0; float accel_z = az / 4096.0; float accel_x = ax / 4096.0;
    acc_roll_sum += (atan2(accel_y, accel_z) * 180 / PI);
    acc_pitch_sum += (atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180 / PI);
    delay(2);
  }
  
  gyro_cal_roll /= 2000.0; gyro_cal_pitch /= 2000.0; gyro_cal_yaw /= 2000.0;
  TARA_ROLL = -(acc_roll_sum / 2000.0);
  TARA_PITCH = -(acc_pitch_sum / 2000.0);

  Serial.print("Auto-TARA ROLL: "); Serial.println(TARA_ROLL);
  Serial.print("Auto-TARA PITCH: "); Serial.println(TARA_PITCH);
  
  myTelemetry.mpu_status = 0; 
  if (controllerKnown) esp_now_send(controllerMAC, (uint8_t *) &myTelemetry, sizeof(myTelemetry));
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  memcpy((void*)&myData, incomingData, sizeof(myData));
  lastRecvTime = millis(); 

  if (!controllerKnown) {
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        memcpy(controllerMAC, info->src_addr, 6);
    #else
        memcpy(controllerMAC, mac, 6);
    #endif
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controllerMAC, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    controllerKnown = true;
  }
  myTelemetry.drone_voltage = battery_voltage;
  esp_now_send(controllerMAC, (uint8_t *) &myTelemetry, sizeof(myTelemetry));
}

float ApplyDeadband(int rc_value, int center, int deadband) {
  if (rc_value > center + deadband) return rc_value - deadband;
  if (rc_value < center - deadband) return rc_value + deadband;
  return center;
}

void batteryMonitorTask(void * pvParameters) {
  for(;;) {
    int raw_vbat = analogRead(VBAT_PIN);
    battery_voltage = (raw_vbat / 4095.0) * 3.3 * 4.2049; 
    vTaskDelay(200 / portTICK_PERIOD_MS); 
  }
}

void flightControlTask(void * pvParameters) {
  static float filtered_rate_roll = 0;
  static float filtered_rate_pitch = 0;
  static float filtered_rate_yaw = 0;
  
  const float pt1_alpha = 0.3; 
  
  static bool is_flying = false;
  static float soft_throttle = 1000.0;
  static bool pid_was_active = false;

  static int runaway_counter = 0;
  static bool emergency_stop = false;
  const float RUNAWAY_OUTPUT_THRESHOLD = 350.0;

  // FIX 1: Ab welchem Gas darf der I-Term ueberhaupt voll arbeiten?
  // Darunter geht die Drohne realistisch nicht in die Luft, also kann der
  // Regler dort auch keinen Fehler ausregeln -> I-Term hart deckeln.
  const int TAKEOFF_THROTTLE = 1400;
  const float GROUND_ITERM_LIMIT = 30.0;

  static uint32_t lastPrintTime = 0;

  for(;;) {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
      
      if (millis() - lastRecvTime > 500) myData.isArmed = false;

      if (!myData.isArmed) {
        emergency_stop = false;
        runaway_counter = 0;
      }

      if (myData.triggerCalib) {
        if (!myData.isArmed && myData.throttle < 1050 && !is_flying) {
          safeCalibrateMPU();
        }
      }

      if (myData.isArmed) {
        if (!is_flying && myData.throttle < 1220) {
          is_flying = true;
          soft_throttle = myData.throttle;
        }
        if (is_flying) {
          soft_throttle = myData.throttle; 
        }
      } else {
        if (is_flying) {
          soft_throttle -= 1.5; 
          if (soft_throttle <= 1000.0) {
            soft_throttle = 1000.0;
            is_flying = false; 
          }
        } else {
          soft_throttle = 1000.0;
        }
      }

      int16_t ax, ay, az, gx, gy, gz;
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        xSemaphoreGive(i2cMutex);
      }

      float gyro_rate_roll  = (gx - gyro_cal_roll) / 65.5; 
      float gyro_rate_pitch = (gy - gyro_cal_pitch) / 65.5;
      float gyro_rate_yaw   = (gz - gyro_cal_yaw) / 65.5;

      float accel_x = ax / 4096.0; float accel_y = ay / 4096.0; float accel_z = az / 4096.0;

      float accel_angle_roll  = (atan2(accel_y, accel_z) * 180 / PI) + TARA_ROLL;
      float accel_angle_pitch = (atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180 / PI) + TARA_PITCH;

      angle_roll  = 0.98 * (angle_roll + (gyro_rate_roll * dt)) + 0.02 * accel_angle_roll;
      angle_pitch = 0.98 * (angle_pitch + (gyro_rate_pitch * dt)) + 0.02 * accel_angle_pitch;

      filtered_rate_roll  = (pt1_alpha * gyro_rate_roll)  + ((1.0 - pt1_alpha) * filtered_rate_roll);
      filtered_rate_pitch = (pt1_alpha * gyro_rate_pitch) + ((1.0 - pt1_alpha) * filtered_rate_pitch);
      filtered_rate_yaw   = (pt1_alpha * gyro_rate_yaw)   + ((1.0 - pt1_alpha) * filtered_rate_yaw);

      rate_in_roll  = filtered_rate_roll;
      rate_in_pitch = filtered_rate_pitch;
      rate_in_yaw   = filtered_rate_yaw;

      float rc_roll_clean  = ApplyDeadband(myData.roll, 1500, RC_DEADBAND);
      float rc_pitch_clean = ApplyDeadband(myData.pitch, 1500, RC_DEADBAND);
      float rc_yaw_clean   = ApplyDeadband(myData.yaw, 1500, RC_DEADBAND);

      float norm_roll  = (rc_roll_clean - 1500) / 500.0;
      float norm_pitch = (rc_pitch_clean - 1500) / 500.0;
      float norm_yaw   = (rc_yaw_clean - 1500) / 500.0;

      float expo_faktor = 0.6;
      float expo_roll  = (norm_roll * norm_roll * norm_roll) * expo_faktor + norm_roll * (1.0 - expo_faktor);
      float expo_pitch = (norm_pitch * norm_pitch * norm_pitch) * expo_faktor + norm_pitch * (1.0 - expo_faktor);
      float expo_yaw   = (norm_yaw * norm_yaw * norm_yaw) * expo_faktor + norm_yaw * (1.0 - expo_faktor);

      float ziel_winkel_roll  = expo_roll * 20.0;   
      float ziel_winkel_pitch = expo_pitch * -20.0; 
      float ziel_rate_yaw     = (expo_yaw * -80.0) + TRIM_YAW;  

      rate_sp_roll  = KP_ANGLE * (ziel_winkel_roll - angle_roll);
      rate_sp_pitch = KP_ANGLE * (ziel_winkel_pitch - angle_pitch);
      rate_sp_yaw   = ziel_rate_yaw;

      int base_throttle = (int)soft_throttle;
      if (is_flying && myData.isArmed) {
        if (base_throttle < MIN_THROTTLE) base_throttle = MIN_THROTTLE; 
      }

      bool pid_active = (base_throttle >= 1195);

      if (!pid_active) {
        rate_sp_roll  = rate_in_roll;
        rate_sp_pitch = rate_in_pitch;
        rate_sp_yaw   = rate_in_yaw;
      }

      pidRoll.Compute();
      pidPitch.Compute();
      pidYaw.Compute();

      // FIX 1: I-Term am Boden hart deckeln (der eigentliche Windup-Stopper).
      if (base_throttle < TAKEOFF_THROTTLE) {
        pidRoll.SetOutputSum(constrain(pidRoll.GetOutputSum(),   -GROUND_ITERM_LIMIT, GROUND_ITERM_LIMIT));
        pidPitch.SetOutputSum(constrain(pidPitch.GetOutputSum(), -GROUND_ITERM_LIMIT, GROUND_ITERM_LIMIT));
        pidYaw.SetOutputSum(constrain(pidYaw.GetOutputSum(),     -GROUND_ITERM_LIMIT, GROUND_ITERM_LIMIT));
      }

      if (!pid_active) {
        pwm_out_roll = 0;
        pwm_out_pitch = 0;
        pwm_out_yaw = 0;
        if (pid_was_active) {
          pidRoll.SetOutputSum(0);
          pidPitch.SetOutputSum(0);
          pidYaw.SetOutputSum(0);
        }
      }
      pid_was_active = pid_active;

      if (is_flying) {
        float max_voltage = 12.6; 
        float vbat_kompensation = 1.0;
        if (battery_voltage > 9.0 && battery_voltage <= max_voltage) {
            vbat_kompensation = max_voltage / battery_voltage;
        }
        pwm_out_roll  *= vbat_kompensation;
        pwm_out_pitch *= vbat_kompensation;
        pwm_out_yaw   *= vbat_kompensation;
      }

      // FIX 2: Nach der VBAT-Multiplikation neu clampen. Vorher konnte die
      // Kompensation das PID-Limit ueberschreiten (Log zeigte Y:-479).
      pwm_out_roll  = constrain(pwm_out_roll,  (float)-PID_LIMIT, (float)PID_LIMIT);
      pwm_out_pitch = constrain(pwm_out_pitch, (float)-PID_LIMIT, (float)PID_LIMIT);
      pwm_out_yaw   = constrain(pwm_out_yaw,   (float)-PID_LIMIT, (float)PID_LIMIT);

      // FIX 3: Runaway-Erkennung mit Zaehler statt hartem Reset.
      bool pid_maxed = (fabs(pwm_out_roll)  > RUNAWAY_OUTPUT_THRESHOLD) ||
                       (fabs(pwm_out_pitch) > RUNAWAY_OUTPUT_THRESHOLD) ||
                       (fabs(pwm_out_yaw)   > RUNAWAY_OUTPUT_THRESHOLD);

      if (pid_active && pid_maxed) {
        if (runaway_counter < 1000) runaway_counter += 2;
      } else if (runaway_counter > 0) {
        runaway_counter--;
      }
      if (runaway_counter > 200) emergency_stop = true;

      int finalMotorFL, finalMotorFR, finalMotorBL, finalMotorBR;

      if (emergency_stop) {
        finalMotorFL = finalMotorFR = finalMotorBL = finalMotorBR = 1000;
        is_flying = false;
        soft_throttle = 1000.0;
        myData.isArmed = false;
      } else if (!is_flying) {
        finalMotorFL = finalMotorFR = finalMotorBL = finalMotorBR = 1000;
      } else {
        int m_FL_raw = base_throttle + pwm_out_pitch + pwm_out_roll + pwm_out_yaw; 
        int m_FR_raw = base_throttle + pwm_out_pitch - pwm_out_roll - pwm_out_yaw; 
        int m_BL_raw = base_throttle - pwm_out_pitch + pwm_out_roll - pwm_out_yaw; 
        int m_BR_raw = base_throttle - pwm_out_pitch - pwm_out_roll + pwm_out_yaw; 

        int max_motor = max(max(m_FL_raw, m_FR_raw), max(m_BL_raw, m_BR_raw));
        if (max_motor > MAX_THROTTLE) {
            int overflow = max_motor - MAX_THROTTLE; 
            m_FL_raw -= overflow;
            m_FR_raw -= overflow;
            m_BL_raw -= overflow;
            m_BR_raw -= overflow;
        }

        finalMotorFL = constrain(m_FL_raw, MIN_THROTTLE, MAX_THROTTLE);
        finalMotorFR = constrain(m_FR_raw, MIN_THROTTLE, MAX_THROTTLE);
        finalMotorBL = constrain(m_BL_raw, MIN_THROTTLE, MAX_THROTTLE);
        finalMotorBR = constrain(m_BR_raw, MIN_THROTTLE, MAX_THROTTLE);
      }

      writeMotor(MOTOR_FL_PIN, finalMotorFL);
      writeMotor(MOTOR_FR_PIN, finalMotorFR);
      writeMotor(MOTOR_BL_PIN, finalMotorBL);
      writeMotor(MOTOR_BR_PIN, finalMotorBR);

      // FIX 4: Erweitertes Log - zeigt jetzt auch Sticks, Winkel, Soll-/Ist-Raten
      // und die I-Terme, um die Quelle des Dauerfehlers zu finden.
      if (millis() - lastPrintTime >= 300) {
        lastPrintTime = millis();
        Serial.printf(
          "Thr:%4d | STICK r:%4d p:%4d y:%4d | WINKEL roll:%6.1f pitch:%6.1f | "
          "SOLL-RATE r:%6.1f p:%6.1f y:%6.1f | IST-RATE r:%6.1f p:%6.1f y:%6.1f | "
          "I-TERM r:%6.1f p:%6.1f y:%6.1f | OUT r:%5.0f p:%5.0f y:%5.0f | "
          "Motoren FL:%4d FR:%4d BL:%4d BR:%4d | Stop:%d\n",
          base_throttle,
          myData.roll, myData.pitch, myData.yaw,
          angle_roll, angle_pitch,
          rate_sp_roll, rate_sp_pitch, rate_sp_yaw,
          rate_in_roll, rate_in_pitch, rate_in_yaw,
          pidRoll.GetOutputSum(), pidPitch.GetOutputSum(), pidYaw.GetOutputSum(),
          pwm_out_roll, pwm_out_pitch, pwm_out_yaw,
          finalMotorFL, finalMotorFR, finalMotorBL, finalMotorBR,
          emergency_stop
        );
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  
  i2cMutex = xSemaphoreCreateMutex(); 

  Wire.begin(I2C_SDA, I2C_SCL); 
  Wire.setClock(400000);
  
  mpu.initialize();
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

  ledcAttach(MOTOR_FL_PIN, 400, 13);
  ledcAttach(MOTOR_FR_PIN, 400, 13);
  ledcAttach(MOTOR_BL_PIN, 400, 13);
  ledcAttach(MOTOR_BR_PIN, 400, 13);

  writeMotor(MOTOR_FL_PIN, 1000);
  writeMotor(MOTOR_FR_PIN, 1000);
  writeMotor(MOTOR_BL_PIN, 1000);
  writeMotor(MOTOR_BR_PIN, 1000);
  delay(3000);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
    #else
        esp_now_register_recv_cb(OnDataRecv);
    #endif
  }

  timerSemaphore = xSemaphoreCreateBinary();
  
  xTaskCreatePinnedToCore(flightControlTask, "FlightTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(batteryMonitorTask, "BatteryTask", 2048, NULL, 1, NULL, 0);

  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000, true, 0); 
}

void loop() {
  vTaskDelete(NULL); 
}
