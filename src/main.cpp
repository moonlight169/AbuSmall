#include "Arduino.h"
#include "Motor.h"
#include "config.h"
#include "Holding.h"
#include "Kinematics.h"
#include <PS4Controller.h>

Motor MotorFL(MotorPinFL_A, MotorPinFL_B, MAX_RPM);
Motor MotorFR(MotorPinFR_A, MotorPinFR_B, MAX_RPM);
Motor MotorRL(MotorPinRL_A, MotorPinRL_B, MAX_RPM);
Motor MotorRR(MotorPinRR_A, MotorPinRR_B, MAX_RPM);

Kinematics kinematics(Kinematics::MECANUM, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

#define COMMAND_RATE 50
unsigned long prev_control_time = 0;

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

float SPEED_SCALE_FACTOR = 0.5; // เริ่มต้นที่ 50%
int speed_mode = 0; 

const int LStickX_Calib = 40;
const int LStickY_Calib = 20;
const int RStickX_Calib = 20;

// ตัวแปรสำหรับควบคุมการหมุนแบบ Differential Steering
bool brake_left = false;
bool brake_right = false;
float brake_factor = 0.3;

Holding Holding;

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  // ใช้สูตร PWM ดั้งเดิมของคุณ (คืนค่า Speed Scale ให้ทำงานถูกต้อง)
  float target_pwm_max = 255.0 * SPEED_SCALE_FACTOR;

  int pwm1 = (req_rpm.motor1 / (float)MAX_RPM) * target_pwm_max;
  int pwm2 = (req_rpm.motor2 / (float)MAX_RPM) * target_pwm_max;
  int pwm3 = (req_rpm.motor3 / (float)MAX_RPM) * target_pwm_max;
  int pwm4 = (req_rpm.motor4 / (float)MAX_RPM) * target_pwm_max;

  // คืนค่าระบบเลี้ยวโค้งด้วยการเบรกล้อ (Differential Steering) กลับมา
  if (brake_left) {
    pwm1 *= brake_factor;  // ล้อหน้าซ้าย
    pwm3 *= brake_factor;  // ล้อหลังซ้าย
  }
  if (brake_right) {
    pwm2 *= brake_factor;  // ล้อหน้าขวา
    pwm4 *= brake_factor;  // ล้อหลังขวา
  }

  MotorFL.run(pwm1);
  MotorFR.run(pwm2);
  MotorRL.run(pwm3);
  MotorRR.run(pwm4);
}

void update_control() {
  if (!PS4.isConnected()) {
    g_req_linear_vel_x = 0; 
    g_req_linear_vel_y = 0; 
    g_req_angular_vel_z = 0;
    brake_left = false;
    brake_right = false;
    return;
  }

  // --- 1. จัดการโหมดความเร็ว (Share Button) ---
  if (PS4.Options()) {
    speed_mode = (speed_mode + 1) % 2;
    if (speed_mode == 0) {
      SPEED_SCALE_FACTOR = 0.5;
    } else {
      SPEED_SCALE_FACTOR = 0.2;
    }
    delay(300);
  }

  float current_max = MAX_RPM * SPEED_SCALE_FACTOR;

  float d_x = 0;
  float d_y = 0;
  float d_z = 0;
  float a_x = 0;
  float a_y = 0;
  float a_z = 0;

  // --- 2. อ่านค่าจาก D-Pad ---
  if (PS4.Up()) {
    d_x = current_max;
  } else if (PS4.Down()) {
    d_x = -current_max;
  }
  
  if (PS4.Left()) {
    d_y = current_max;
  } else if (PS4.Right()) {
    d_y = -current_max;
  }

  // อ่านค่า L1/R1 เบื้องต้น
  brake_left = PS4.L1();
  brake_right = PS4.R1();

  // --- 3. อ่านค่าจาก Analog Stick พร้อมตัด Deadzone ---
  int L_X = PS4.LStickX();
  int L_Y = PS4.LStickY();
  int R_X = PS4.RStickX();

  if (abs(L_Y) > LStickY_Calib) {
    a_x = map(L_Y, -128, 127, -current_max, current_max);
  }
  if (abs(R_X) > RStickX_Calib) {
    a_y = map(R_X, 127, -128, -current_max, current_max);
  }
  if (abs(L_X) > LStickX_Calib) {
    a_z = map(L_X, 127, -128, -current_max, current_max);
  }

  // --- 4. รวมค่า X, Y ---
  g_req_linear_vel_x = a_x + d_x;
  g_req_linear_vel_y = a_y + d_y;
  
  // --- 5. L1/R1 Smart Mode ดั้งเดิม ---
  bool is_moving = (abs(g_req_linear_vel_x) > 10 || abs(g_req_linear_vel_y) > 10);
  
  if (is_moving) {
    // เดินอยู่: ให้เบรกทำงานใน moveBase
    g_req_angular_vel_z = a_z; 
  } else {
    // ไม่เดิน: ให้ L1/R1 หมุนตัวอยู่กับที่
    if (PS4.L1()) {
      d_z = current_max;  
    } else if (PS4.R1()) {
      d_z = -current_max; 
    }
    g_req_angular_vel_z = a_z + d_z;
    
    // ปิดสถานะเบรก เพื่อให้หมุนตัวได้เต็มกำลัง
    brake_left = false;
    brake_right = false;
  }

  // --- 6. ป้องกันค่าเกินขอบเขต ---
  g_req_linear_vel_x = constrain(g_req_linear_vel_x, -current_max, current_max);
  g_req_linear_vel_y = constrain(g_req_linear_vel_y, -current_max, current_max);
  g_req_angular_vel_z = constrain(g_req_angular_vel_z, -current_max, current_max);
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  PS4.begin("d0:ef:76:ed:f9:7c");
  Holding.init();
}

void loop() {
  update_control();

  unsigned long now = millis();
  if ((now - prev_control_time) >= (1000 / COMMAND_RATE)) {
    moveBase();
    prev_control_time = now;
  }
}