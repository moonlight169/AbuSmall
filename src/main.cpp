#include "Arduino.h"
#include "Motor.h"
#include "config.h"
#include "Holding.h"
#include "Kinematics.h"
#include <PS4Controller.h>

// สร้าง Object มอเตอร์ และ Kinematics (ตัด PID ออกตามที่คุยกันเพื่อให้ Speed Scale ทำงาน)
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

float SPEED_SCALE_FACTOR = 0.5; // เริ่มต้นที่ 50% ของความเร็วสูงสุด
int speed_mode = 0; 

const int LStickX_Calib = 40;
const int LStickY_Calib = 20;
const int RStickX_Calib = 20;

// ตัวแปรสำหรับควบคุมการหมุนแบบ Differential Steering
bool brake_left = false;   // L1 = ล้อซ้ายช้าลง
bool brake_right = false;  // R1 = ล้อขวาช้าลง
float brake_factor = 0.3;  // ลดแรง 30% เมื่อกด L1/R1

Holding Holding;

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  // --- วิธีแก้แบบคุม PWM โดยตรง ---
  // ให้เอาค่า RPM ที่ได้ หารด้วย MAX_RPM เพื่อหาเป็น % (0.0 - 1.0)
  // แล้วคูณด้วย 255 (PWM สูงสุด) และคูณด้วย SPEED_SCALE_FACTOR อีกที
  
  float target_pwm_max = 255.0 * SPEED_SCALE_FACTOR;

  int pwm1 = (req_rpm.motor1 / (float)MAX_RPM) * target_pwm_max;
  int pwm2 = (req_rpm.motor2 / (float)MAX_RPM) * target_pwm_max;
  int pwm3 = (req_rpm.motor3 / (float)MAX_RPM) * target_pwm_max;
  int pwm4 = (req_rpm.motor4 / (float)MAX_RPM) * target_pwm_max;

  // --- ปรับความเร็วเมื่อกด L1/R1 เพื่อหมุน ---
  // L1 = ล้อซ้ายช้าลง (FL, RL) ทำให้หมุนไปขวา
  // R1 = ล้อขวาช้าลง (FR, RR) ทำให้หมุนไปซ้าย
  if (brake_left) {
    pwm1 *= brake_factor;  // Front Left
    pwm3 *= brake_factor;  // Rear Left
  }
  if (brake_right) {
    pwm2 *= brake_factor;  // Front Right
    pwm4 *= brake_factor;  // Rear Right
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
    return;
  }

  // --- 1. จัดการโหมดความเร็ว (Share Button) ---
  if (PS4.Share()) {
    speed_mode = (speed_mode + 1) % 2;
    SPEED_SCALE_FACTOR = (speed_mode == 0) ? 0.5 : 0.2; // ปกติ 50%, ช้า 20%
    delay(300);
  }

  // ตัวแปรชั่วคราวสำหรับคำนวณ
  float d_x = 0, d_y = 0, d_z = 0; // จาก D-Pad/Buttons
  float a_x = 0, a_y = 0, a_z = 0; // จาก Analog
  float current_max = MAX_RPM * SPEED_SCALE_FACTOR;

  // --- 2. อ่านค่าจาก D-Pad และ L1/R1 (ปุ่มกด) ---
  if (PS4.Up())         d_x = current_max;
  else if (PS4.Down())  d_x = -current_max;
  
  // Left/Right = สไลด์ (lateral movement)
  if (PS4.Left())       d_y = current_max;
  else if (PS4.Right()) d_y = -current_max;

  // L1/R1 = หมุนโดยลดแรงล้อ (Differential Steering) หรือ Spin ตั้งแต่เดิม
  // L1 = ล้อซ้ายช้าลง (หมุนไปขวา)
  // R1 = ล้อขวาช้าลง (หมุนไปซ้าย)
  brake_left = PS4.L1();
  brake_right = PS4.R1();

  // --- 3. อ่านค่าจาก Analog Stick ---
  int L_X = PS4.LStickX();
  int L_Y = PS4.LStickY();
  int R_X = PS4.RStickX();

  // ตรวจสอบ Deadzone ก่อนคำนวณ Analog
  if (abs(L_Y) > LStickY_Calib) {
    a_x = map(L_Y, -128, 127, -current_max, current_max);
  }
  if (abs(R_X) > RStickX_Calib) {
    a_y = map(R_X, 127, -128, -current_max, current_max);
  }
  // Left Stick X ไม่ใช้ เพราะ L1/R1 ทำการหมุนแล้ว
  if (abs(L_X) > LStickX_Calib) {
    a_z = map(L_X, 127, -128, -current_max, current_max);
  }

  // --- 4. รวมค่า (Combine) เพื่อให้ทำงานพร้อมกันได้ ---
  // การบวกกันตรงๆ จะทำให้คุณกดทั้งปุ่มและดัน Analog เพื่อเพิ่มความแรง หรือทำงานคนละแกนพร้อมกันได้
  g_req_linear_vel_x = a_x + d_x;
  g_req_linear_vel_y = a_y + d_y;
  
  // --- L1/R1 Smart Mode ---
  // ถ้าเดินอยู่: ใช้ Differential Steering (ลดแรงล้อ)
  // ถ้าไม่เดิน: ใช้ Spin และท้าย (หมุนตัวในที่)
  bool is_moving = (abs(g_req_linear_vel_x) > 10 || abs(g_req_linear_vel_y) > 10);
  
  if (is_moving) {
    // เดินอยู่: ใช้ Differential Steering (brake_left/brake_right ใน moveBase)
    g_req_angular_vel_z = a_z;
  } else {
    // ไม่เดิน: L1/R1 ให้หมุนตัวตรง
    // L1 = หมุนซ้าย (Counter-Clockwise)
    // R1 = หมุนขวา (Clockwise)
    if (PS4.L1()) {
      d_z = current_max;  // หมุนซ้าย
    } else if (PS4.R1()) {
      d_z = -current_max; // หมุนขวา
    }
    g_req_angular_vel_z = a_z + d_z;
  }

  // --- 5. ป้องกันค่าเกินขอบเขต (Constrain) ---
  // ป้องกันกรณีที่บวกกันแล้วค่าพุ่งเกิน MAX_RPM จน Kinematics คำนวณเพี้ยน
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