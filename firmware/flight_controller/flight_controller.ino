// Flight controller for a self-built ESP32 quadcopter.
//
// Hardware: ESP32 DevKitC (WROOM-32U), MPU6050 over I2C, 4-in-1 BLHeli_S ESC
// driven with 400 Hz PWM, 3S LiPo. Commands come from the custom remote
// control over ESP-NOW.
//
// Structure:
//   flightControlTask   core 1, 1 kHz from a hardware timer, highest priority
//   batteryMonitorTask  core 0, battery voltage every 200 ms
//   telemetryTask       core 0, sends voltage, calibration state and gyro
//                       drift back to the remote every 100 ms
//
// Control: angle loop (P) -> rate loop (PID, QuickPID) per axis, complementary
// filter for roll/pitch, heading hold on yaw, airmode mixer.

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include "QuickPID.h"

// ---------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------
const int MOTOR_BR_PIN = 26;
const int MOTOR_FR_PIN = 25;
const int MOTOR_BL_PIN = 27;
const int MOTOR_FL_PIN = 32;

const int I2C_SDA = 21;
const int I2C_SCL = 23;

const int VBAT_PIN = 34;

// ---------------------------------------------------------------------
// Throttle and safety settings (ESC pulse width in us)
// ---------------------------------------------------------------------
const int MIN_THROTTLE     = 1180;   // motors run smoothly from here
const int MAX_THROTTLE     = 2000;
const int ARM_THROTTLE_MAX = 1220;   // arming is only accepted below this
const int PID_ACTIVE_THR   = 1195;   // PID output is ignored below this

// Once throttle has been above this value, the I-term is released until
// disarm. Has to be below hover throttle (3S setup lifts off at about
// 30-40 %, i.e. 1426-1508), otherwise the I-term stays clamped in flight.
// Has to be above idle so the windup limit still works on the ground.
// 1280 was too low: the I-term was already free while the drone was rocking
// on its legs and made the rocking worse.
const int TAKEOFF_THROTTLE = 1350;

// The sticks do not return exactly to 1500 (measured at rest: roll 1465,
// pitch 1538, yaw 1538). 45 covers that offset and costs about 9 % travel.
const int RC_DEADBAND      = 45;
const int PID_LIMIT        = 400;

const uint32_t FAILSAFE_MS = 500;

// Complementary filter time constant in seconds. Higher means more gyro and
// less accelerometer. Was 0.049 s before (alpha 0.98), which let vibration
// through as a fake tilt.
const float COMP_TAU = 1.0f;

// Throttle ramp after disarm in us per ms. 0.4 takes about 2.5 s from full
// throttle, so a panic disarm in the air does not drop the drone.
const float LANDING_RAMP = 0.4f;

// Maximum rate setpoint the angle loop may request (deg/s)
const float MAX_RATE_SP = 250.0f;

// Yaw heading hold. With the yaw stick centred the drone holds the heading
// it had when the stick was released, instead of only holding zero rate
// (which let gyro error turn it slowly). Gyro drift remains, there is no
// magnetometer.
const bool  USE_YAW_HOLD      = true;
const float KP_YAW_ANGLE      = 3.0f;    // too high: jerks on stick release
const float YAW_STICK_MIN     = 2.0f;    // deg/s, above this = rate mode
const float YAW_HOLD_MAX_RATE = 120.0f;  // limits snap back after a push

// Airmode keeps the difference between the motors and shifts the common
// throttle instead of clipping single motors at idle. Without it the drone
// lost authority right at take-off. Costs a little extra thrust during hard
// corrections. Set to false for the old mixer.
const bool USE_AIRMODE = true;

const uint32_t LOG_INTERVAL_MS = 300;

// Serial log:
//   0 = off
//   1 = full status line every 300 ms
//   2 = angle, rate and output of one axis at 100 Hz for the Arduino
//       Serial Plotter (used for PID tuning)
#define LOG_MODE 1
#define LOG_AXIS 0   // 0 = roll, 1 = pitch

const uint32_t LOG_FAST_INTERVAL_MS = 10;

// ---------------------------------------------------------------------
// Data structures (must match the remote control exactly)
// ---------------------------------------------------------------------
typedef struct struct_message {
  int  throttle;
  int  yaw;
  int  roll;
  int  pitch;
  bool isArmed;
  bool triggerCalib;
} struct_message;

// gyro_drift: yaw rate measured at standstill, should be close to 0.
// mpu_temp:   chip temperature, calibrate once it stops rising.
typedef struct struct_telemetry {
  float drone_voltage;
  int   mpu_status;
  float gyro_drift;
  float mpu_temp;
} struct_telemetry;

// Shared with the receive callback, only access inside rxMux
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

float ACC_OFFSET_PITCH = 0.0;
float ACC_OFFSET_ROLL  = 0.0;
float TRIM_YAW         = 0.0;

// Permanent trim in degrees. Calibrate on a level surface first, use this
// only for what is left. Sign depends on how the board is mounted.
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

// Tuned for the 3S / 94 mm arm setup, about 30 % below the old 2S / 134 mm
// values (higher voltage and lower inertia give more angular acceleration
// for the same output).
//   fast jitter / buzzing        -> lower Kd first, then Kp
//   slow oscillation             -> lower Kp
//   tilts away and stays there   -> raise Ki
//   drifts in one direction only -> recalibrate on a level surface
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
// Motors
// ---------------------------------------------------------------------
void writeMotor(int pin, int us) {
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  uint32_t duty = ((uint32_t)us * 8192UL) / 2500UL;   // 400 Hz = 2500 us period, 13 bit
  ledcWrite(pin, duty);
}

void allMotors(int us) {
  writeMotor(MOTOR_FL_PIN, us);
  writeMotor(MOTOR_FR_PIN, us);
  writeMotor(MOTOR_BL_PIN, us);
  writeMotor(MOTOR_BR_PIN, us);
}

// Called first in setup(), so the ESC inputs are not floating during the
// few seconds of IMU calibration.
void initMotorsSafe() {
  ledcAttach(MOTOR_FL_PIN, 400, 13);
  ledcAttach(MOTOR_FR_PIN, 400, 13);
  ledcAttach(MOTOR_BL_PIN, 400, 13);
  ledcAttach(MOTOR_BR_PIN, 400, 13);
  allMotors(1000);
}

// ---------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------
void safeCalibrateMPU() {
  myTelemetry.mpu_status = 1;   // shown as "CAL" on the remote

  float acc_roll_sum  = 0;
  float acc_pitch_sum = 0;
  gyro_cal_roll = 0; gyro_cal_pitch = 0; gyro_cal_yaw = 0;

  Serial.println("Calibrating gyro and accelerometer, do not move...");

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
  ACC_OFFSET_ROLL  = -(acc_roll_sum  / 2000.0f);
  ACC_OFFSET_PITCH = -(acc_pitch_sum / 2000.0f);

  // restart the filter from level
  angle_roll  = 0.0f;
  angle_pitch = 0.0f;

  Serial.print("Accel offset roll: ");  Serial.println(ACC_OFFSET_ROLL);
  Serial.print("Accel offset pitch: "); Serial.println(ACC_OFFSET_PITCH);

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
  if (len != (int)sizeof(struct_message)) return;

  struct_message tmp;
  memcpy(&tmp, incomingData, sizeof(tmp));

  portENTER_CRITICAL(&rxMux);
  rxData   = tmp;
  rxLastMs = millis();
  portEXIT_CRITICAL(&rxMux);

  // The callback runs in the WiFi task, so adding the peer and sending
  // telemetry is left to telemetryTask. Here we only remember the sender.
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
      // temperature about once per second, with a timeout so this task
      // never blocks on the I2C mutex
      static uint8_t tempDivider = 0;
      if (++tempDivider >= 10) {
        tempDivider = 0;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          int16_t traw = mpu.getTemperature();
          xSemaphoreGive(i2cMutex);
          myTelemetry.mpu_temp = traw / 340.0f + 36.53f;   // datasheet formula
        }
      }

      myTelemetry.drone_voltage = battery_voltage;
      esp_now_send(controllerMAC, (uint8_t *) &myTelemetry, sizeof(myTelemetry));
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------
float ApplyDeadband(int rc_value, int center, int deadband) {
  if (rc_value > center + deadband) return rc_value - deadband;
  if (rc_value < center - deadband) return rc_value + deadband;
  return center;
}

void batteryMonitorTask(void *pvParameters) {
  // divider 31.6k / 9.86k
  battery_voltage = (analogRead(VBAT_PIN) / 4095.0f) * 3.3f * 4.2049f;
  for (;;) {
    float raw = (analogRead(VBAT_PIN) / 4095.0f) * 3.3f * 4.2049f;
    battery_voltage = battery_voltage * 0.8f + raw * 0.2f;   // smooth ADC noise
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------
void flightControlTask(void *pvParameters) {
  static float filtered_rate_roll = 0, filtered_rate_pitch = 0, filtered_rate_yaw = 0;
  const float pt1_alpha = 0.3f;

  static bool  is_flying      = false;
  static float soft_throttle  = 1000.0f;
  static bool  pid_was_active = false;
  static bool  has_taken_off  = false;

  static float yaw_heading     = 0.0f;   // integrated yaw angle (deg)
  static float yaw_hold_target = 0.0f;

  static int  runaway_counter = 0;
  static bool emergency_stop  = false;
  const float RUNAWAY_OUTPUT_THRESHOLD = 350.0f;

  const float GROUND_ITERM_LIMIT = 30.0f;

  const float comp_alpha = COMP_TAU / (COMP_TAU + dt);

  static uint32_t lastPrintTime = 0;
  static uint32_t lastArmWarn   = 0;

  for (;;) {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) != pdTRUE) continue;

    // ---------- copy RX data in one piece ----------
    struct_message cmd;
    uint32_t lastRx;
    portENTER_CRITICAL(&rxMux);
    cmd    = rxData;
    lastRx = rxLastMs;
    portEXIT_CRITICAL(&rxMux);

    bool armed_from_rx = cmd.isArmed;
    bool link_ok = (millis() - lastRx) <= FAILSAFE_MS;

    // ---------- failsafe: disarm and centre the sticks ----------
    if (!link_ok) {
      cmd.isArmed      = false;
      cmd.triggerCalib = false;
      cmd.roll  = 1500;
      cmd.pitch = 1500;
      cmd.yaw   = 1500;
      armed_from_rx = false;
    }

    // emergency stop is only reset by a real disarm from the pilot
    if (!armed_from_rx) {
      emergency_stop  = false;
      runaway_counter = 0;
      has_taken_off   = false;
    }

    // ---------- calibration ----------
    if (cmd.triggerCalib && !cmd.isArmed && cmd.throttle < 1050 && !is_flying) {
      allMotors(1000);
      safeCalibrateMPU();
      continue;   // this cycle took too long, start fresh
    }

    // ---------- arming / throttle ramp ----------
    if (cmd.isArmed) {
      if (!is_flying) {
        if (cmd.throttle < ARM_THROTTLE_MAX) {
          is_flying     = true;
          soft_throttle = cmd.throttle;
        } else if (millis() - lastArmWarn > 1000) {
          lastArmWarn = millis();
          Serial.printf("Arming refused: throttle too high (%d, needs < %d)\n",
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

    // ---------- IMU ----------
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      xSemaphoreGive(i2cMutex);
    }

    float gyro_rate_roll  = (gx - gyro_cal_roll)  / 65.5f;   // +-500 deg/s range
    float gyro_rate_pitch = (gy - gyro_cal_pitch) / 65.5f;
    float gyro_rate_yaw   = (gz - gyro_cal_yaw)   / 65.5f;

    float accel_x = ax / 4096.0f, accel_y = ay / 4096.0f, accel_z = az / 4096.0f;   // +-8 g

    float accel_angle_roll  = (atan2(accel_y, accel_z) * 180.0 / PI) + ACC_OFFSET_ROLL;
    float accel_angle_pitch = (atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI) + ACC_OFFSET_PITCH;

    // complementary filter
    angle_roll  = comp_alpha * (angle_roll  + gyro_rate_roll  * dt) + (1.0f - comp_alpha) * accel_angle_roll;
    angle_pitch = comp_alpha * (angle_pitch + gyro_rate_pitch * dt) + (1.0f - comp_alpha) * accel_angle_pitch;

    // yaw angle from the gyro only, drifts slowly but is good enough
    // to hold a heading for one flight
    yaw_heading += gyro_rate_yaw * dt;

    // while on the ground every yaw rate is error, send it to the remote
    if (!is_flying) {
      static float drift_est = 0.0f;
      drift_est = drift_est * 0.999f + gyro_rate_yaw * 0.001f;
      myTelemetry.gyro_drift = drift_est;
    }

    // PT1 low pass so the D-term does not react to sensor noise
    filtered_rate_roll  = (pt1_alpha * gyro_rate_roll)  + ((1.0f - pt1_alpha) * filtered_rate_roll);
    filtered_rate_pitch = (pt1_alpha * gyro_rate_pitch) + ((1.0f - pt1_alpha) * filtered_rate_pitch);
    filtered_rate_yaw   = (pt1_alpha * gyro_rate_yaw)   + ((1.0f - pt1_alpha) * filtered_rate_yaw);

    rate_in_roll  = filtered_rate_roll;
    rate_in_pitch = filtered_rate_pitch;
    rate_in_yaw   = filtered_rate_yaw;

    // ---------- stick input ----------
    float rc_roll_clean  = ApplyDeadband(constrain(cmd.roll,  1000, 2000), 1500, RC_DEADBAND);
    float rc_pitch_clean = ApplyDeadband(constrain(cmd.pitch, 1000, 2000), 1500, RC_DEADBAND);
    float rc_yaw_clean   = ApplyDeadband(constrain(cmd.yaw,   1000, 2000), 1500, RC_DEADBAND);

    float norm_roll  = (rc_roll_clean  - 1500) / 500.0f;
    float norm_pitch = (rc_pitch_clean - 1500) / 500.0f;
    float norm_yaw   = (rc_yaw_clean   - 1500) / 500.0f;

    // 60 % cubic, 40 % linear: finer control around centre
    const float expo_factor = 0.6f;
    float expo_roll  = (norm_roll  * norm_roll  * norm_roll)  * expo_factor + norm_roll  * (1.0f - expo_factor);
    float expo_pitch = (norm_pitch * norm_pitch * norm_pitch) * expo_factor + norm_pitch * (1.0f - expo_factor);
    float expo_yaw   = (norm_yaw   * norm_yaw   * norm_yaw)   * expo_factor + norm_yaw   * (1.0f - expo_factor);

    float target_angle_roll  = expo_roll  *  20.0f + TRIM_ROLL_DEG;    // +-20 deg
    float target_angle_pitch = expo_pitch * -20.0f + TRIM_PITCH_DEG;
    float stick_rate_yaw     = (expo_yaw  * -80.0f) + TRIM_YAW;        // +-80 deg/s

    // angle loop. constrain() is a macro, so calculate first, then limit
    float sp_r = KP_ANGLE * (target_angle_roll  - angle_roll);
    float sp_p = KP_ANGLE * (target_angle_pitch - angle_pitch);

    // ---------- yaw: rate mode while the stick is moved, heading hold otherwise ----------
    float sp_y;
    if (!USE_YAW_HOLD || !is_flying) {
      sp_y = stick_rate_yaw;
      yaw_heading     = 0.0f;
      yaw_hold_target = 0.0f;
    } else if (fabs(stick_rate_yaw) > YAW_STICK_MIN) {
      sp_y = stick_rate_yaw;
      yaw_hold_target = yaw_heading;
    } else {
      float yaw_err = yaw_hold_target - yaw_heading;
      sp_y = constrain(KP_YAW_ANGLE * yaw_err, -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);
    }

    rate_sp_roll  = constrain(sp_r, -MAX_RATE_SP, MAX_RATE_SP);
    rate_sp_pitch = constrain(sp_p, -MAX_RATE_SP, MAX_RATE_SP);
    rate_sp_yaw   = constrain(sp_y, -MAX_RATE_SP, MAX_RATE_SP);

    // ---------- throttle / PID enable ----------
    int base_throttle = (int)soft_throttle;
    if (is_flying && cmd.isArmed && base_throttle < MIN_THROTTLE) base_throttle = MIN_THROTTLE;

    bool pid_active = (base_throttle >= PID_ACTIVE_THR);

    if (!pid_active) {
      rate_sp_roll  = rate_in_roll;
      rate_sp_pitch = rate_in_pitch;
      rate_sp_yaw   = rate_in_yaw;
    }

    if (base_throttle >= TAKEOFF_THROTTLE) has_taken_off = true;

    if (!has_taken_off) {
      // limit the I-term on the ground before Compute(), so this cycle's
      // output is already limited
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

    // ---------- battery voltage compensation ----------
    if (is_flying) {
      const float max_voltage = 12.6f;
      float vbat_comp = 1.0f;
      if (battery_voltage > 9.0f && battery_voltage <= max_voltage) {
        vbat_comp = max_voltage / battery_voltage;
      }
      pwm_out_roll  *= vbat_comp;
      pwm_out_pitch *= vbat_comp;
      pwm_out_yaw   *= vbat_comp;
    }

    pwm_out_roll  = constrain(pwm_out_roll,  (float)-PID_LIMIT, (float)PID_LIMIT);
    pwm_out_pitch = constrain(pwm_out_pitch, (float)-PID_LIMIT, (float)PID_LIMIT);
    pwm_out_yaw   = constrain(pwm_out_yaw,   (float)-PID_LIMIT, (float)PID_LIMIT);

    // ---------- runaway detection ----------
    // If the drone is stuck (test rig, grass) the I-term winds up until the
    // output saturates. Cut the motors before that is released at once.
    bool pid_maxed = (fabs(pwm_out_roll)  > RUNAWAY_OUTPUT_THRESHOLD) ||
                     (fabs(pwm_out_pitch) > RUNAWAY_OUTPUT_THRESHOLD) ||
                     (fabs(pwm_out_yaw)   > RUNAWAY_OUTPUT_THRESHOLD);

    if (pid_active && pid_maxed) {
      if (runaway_counter < 1000) runaway_counter += 2;
    } else if (runaway_counter > 0) {
      runaway_counter--;
    }
    if (runaway_counter > 200) emergency_stop = true;

    // ---------- mixer (X configuration) ----------
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

      // difference between the motors, without base throttle
      int mix_FL =  p + r + y;
      int mix_FR =  p - r - y;
      int mix_BL = -p + r - y;
      int mix_BR = -p - r + y;

      if (USE_AIRMODE && pid_active) {
        int mix_max = max(max(mix_FL, mix_FR), max(mix_BL, mix_BR));
        int mix_min = min(min(mix_FL, mix_FR), min(mix_BL, mix_BR));
        int mix_range = mix_max - mix_min;
        const int allowed = MAX_THROTTLE - MIN_THROTTLE;

        // 1. if the spread does not fit into the throttle range, scale all
        //    four down evenly so the ratio between the axes stays the same
        if (mix_range > allowed && mix_range > 0) {
          float scale = (float)allowed / (float)mix_range;
          mix_FL = lroundf(mix_FL * scale);
          mix_FR = lroundf(mix_FR * scale);
          mix_BL = lroundf(mix_BL * scale);
          mix_BR = lroundf(mix_BR * scale);
          mix_max = max(max(mix_FL, mix_FR), max(mix_BL, mix_BR));
          mix_min = min(min(mix_FL, mix_FR), min(mix_BL, mix_BR));
        }

        // 2. shift the base throttle so no motor hits a limit
        int base = base_throttle;
        if (base + mix_min < MIN_THROTTLE) base = MIN_THROTTLE - mix_min;
        if (base + mix_max > MAX_THROTTLE) base = MAX_THROTTLE - mix_max;

        finalMotorFL = base + mix_FL;
        finalMotorFR = base + mix_FR;
        finalMotorBL = base + mix_BL;
        finalMotorBR = base + mix_BR;
      } else {
        // old mixer: only shifts down when a motor is above the maximum
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

      finalMotorFL = constrain(finalMotorFL, MIN_THROTTLE, MAX_THROTTLE);
      finalMotorFR = constrain(finalMotorFR, MIN_THROTTLE, MAX_THROTTLE);
      finalMotorBL = constrain(finalMotorBL, MIN_THROTTLE, MAX_THROTTLE);
      finalMotorBR = constrain(finalMotorBR, MIN_THROTTLE, MAX_THROTTLE);
    }

    writeMotor(MOTOR_FL_PIN, finalMotorFL);
    writeMotor(MOTOR_FR_PIN, finalMotorFR);
    writeMotor(MOTOR_BL_PIN, finalMotorBL);
    writeMotor(MOTOR_BR_PIN, finalMotorBR);

    // ---------- logging (skipped if the TX buffer is full, never blocks) ----------
#if LOG_MODE == 2
    if (millis() - lastPrintTime >= LOG_FAST_INTERVAL_MS && Serial.availableForWrite() > 60) {
      lastPrintTime = millis();
  #if LOG_AXIS == 0
      Serial.printf("angle:%.1f,rate:%.1f,out:%.0f\n",
                    angle_roll, rate_in_roll, pwm_out_roll);
  #else
      Serial.printf("angle:%.1f,rate:%.1f,out:%.0f\n",
                    angle_pitch, rate_in_pitch, pwm_out_pitch);
  #endif
    }
#endif

#if LOG_MODE == 1
    if (millis() - lastPrintTime >= LOG_INTERVAL_MS && Serial.availableForWrite() > 260) {
      lastPrintTime = millis();
      Serial.printf(
        "Thr:%4d | STICK r:%4d p:%4d y:%4d | ANGLE roll:%6.1f pitch:%6.1f | "
        "RATE SP r:%6.1f p:%6.1f y:%6.1f | RATE r:%6.1f p:%6.1f y:%6.1f | "
        "I-TERM r:%6.1f p:%6.1f y:%6.1f | OUT r:%5.0f p:%5.0f y:%5.0f | "
        "Motors FL:%4d FR:%4d BL:%4d BR:%4d | Link:%d TO:%d Stop:%d\n",
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
// Setup
// ---------------------------------------------------------------------
void setup() {
  initMotorsSafe();   // first, before anything that takes time

  Serial.setTxBufferSize(2048);   // so printf in the control loop does not block
  Serial.begin(115200);

  analogReadResolution(12);

  i2cMutex = xSemaphoreCreateMutex();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  mpu.initialize();

  // no IMU, no flight
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not found. Motors stay off.");
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
  delay(3000);   // ESC startup

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed. Motors stay off.");
    for (;;) { allMotors(1000); delay(500); }
  }

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  #else
      esp_now_register_recv_cb(OnDataRecv);
  #endif

  timerSemaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(flightControlTask,  "FlightTask",  10000, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(batteryMonitorTask, "BatteryTask",  2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(telemetryTask,      "TelemTask",    3072, NULL, 2, NULL, 0);

  timer = timerBegin(1000000);          // 1 MHz
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000, true, 0);     // every 1000 us = 1 kHz
}

void loop() {
  vTaskDelete(NULL);   // everything runs in tasks
}
