#include "Arduino.h"
#include "Motor.h"
#include "config.h"
#include "Holding.h"
#include "Kinematics.h"
#include <PS4Controller.h>
#include <Wire.h>

#define Address_Small 0x04

Motor MotorFL(MotorPinFLM1_A, MotorPinFLM1_B, M_MAX_RPM);
Motor MotorFR(MotorPinFRM1_A, MotorPinFRM1_B, M_MAX_RPM);
Motor MotorRL(MotorPinRLM1_A, MotorPinRLM1_B, M_MAX_RPM);
Motor MotorRR(MotorPinRRM1_A, MotorPinRRM1_B, M_MAX_RPM);

Kinematics kinematics(Kinematics::MECANUM, M_MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

#define COMMAND_RATE 50
unsigned long prev_control_time = 0;

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

float f_walkspeed = 1.0;
float n_walkspeed = 0.5;

float f_turnspeed = 1.5;
float n_turnspeed = 1.0;

float f_slidespeed = 0.8;
float n_slidespeed = 0.5;
//------------------------------------
float walkspeed = f_walkspeed;
float turnspeed = f_turnspeed;
float slidespeed = f_slidespeed;

int speed_mode = 0; 

bool last_options_state = false; 

bool last_circle_state = false;
bool last_square_state = false;
bool last_triangle_state = false;
bool last_x_state = false;
bool last_share_state = false;
bool last_L2_state = false;
bool last_r2_state = false;

char last_lift_state = 'S'; 

const int LStickX_Calib = 40;
const int LStickY_Calib = 20;
const int RStickX_Calib = 20;

Holding Holding;

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  // // --- ลอจิกเพิ่มความเร็วตามทิศทาง ---
  // float rpm_offset = 3.0;

  // if (g_req_linear_vel_x > 0) { 
  //   req_rpm.motor1 += rpm_offset; // MotorFL
  //   req_rpm.motor2 += rpm_offset; // MotorFR
  // } 
  // else if (g_req_linear_vel_x < 0) {
  //   req_rpm.motor3 -= rpm_offset; // MotorRL
  //   req_rpm.motor4 -= rpm_offset; // MotorRR
  // }
  // if (g_req_linear_vel_y > 0) { 
  //   req_rpm.motor1 += rpm_offset; // MotorFL
  //   req_rpm.motor4 += rpm_offset; // MotorRR
  // } 
  // else if (g_req_linear_vel_y < 0) {
  //   req_rpm.motor2 -= rpm_offset; // MotorFR
  //   req_rpm.motor3 -= rpm_offset; // MotorRL
  // }

  MotorFL.runRPM(req_rpm.motor1);
  MotorFR.runRPM(req_rpm.motor2);
  MotorRL.runRPM(req_rpm.motor3);
  MotorRR.runRPM(req_rpm.motor4);
}

void update_control() {
  if (!PS4.isConnected()) {
    g_req_linear_vel_x = 0; 
    g_req_linear_vel_y = 0; 
    g_req_angular_vel_z = 0;
    MotorFL.run(0); MotorFR.run(0); MotorRL.run(0); MotorRR.run(0);
    return;
  }

  // --- ระบบสลับความเร็ว (Speed Mode) ---
  bool current_options_state = PS4.Options();
  if (current_options_state && !last_options_state) {
    speed_mode = (speed_mode + 1) % 2;
    if (speed_mode == 0) {
      walkspeed = f_walkspeed;
      turnspeed = f_turnspeed;
      slidespeed = f_slidespeed;
    } else {
      walkspeed = n_walkspeed;
      turnspeed = n_turnspeed;
      slidespeed = n_slidespeed;
    }
  }
  last_options_state = current_options_state;

  float walk_speed = walkspeed;
  float turn_speed = turnspeed;
  float slide_speed = slidespeed;

  float d_x = 0;
  float d_y = 0;
  float d_z = 0;

  // --- ควบคุมการเดินหน้า / ถอยหลัง (D-Pad บน/ล่าง) ---
  if (PS4.Up()) {
    d_x = walk_speed;
  } else if (PS4.Down()) {
    d_x = -walk_speed;
  }
  
  // --- ควบคุมการสไลด์ซ้าย / ขวา (D-Pad ซ้าย/ขวา) ---
  if (PS4.Left()) {
    d_y = slide_speed;
  } else if (PS4.Right()) {
    d_y = -slide_speed;
  }

  // --- ควบคุมการหมุนตัว (L1 / R1) ---
  if (PS4.L1()) {
    d_z = turn_speed;  // L1 = หมุนตัวทวนเข็มนาฬิกา
  } else if (PS4.R1()) {
    d_z = -turn_speed; // R1 = หมุนตัวตามเข็มนาฬิกา
  }

  // กำหนดค่าความเร็วที่จะส่งไปให้สมการ Kinematics
  g_req_linear_vel_x = d_x;
  g_req_linear_vel_y = d_y;
  g_req_angular_vel_z = d_z;

  // Constrain ป้องกันค่าเกิน (เผื่อไว้)
  g_req_linear_vel_x = constrain(g_req_linear_vel_x, -walk_speed, walk_speed);
  g_req_linear_vel_y = constrain(g_req_linear_vel_y, -slide_speed, slide_speed);
  g_req_angular_vel_z = constrain(g_req_angular_vel_z, -turn_speed, turn_speed);
}

void digital_control(){
  bool circle_pressed = PS4.Circle();
  if (circle_pressed && !last_circle_state) {
    digitalWrite(RelayM1_PIN1, !digitalRead(RelayM1_PIN1)); 
  }
  last_circle_state = circle_pressed;

  bool x_pressed = PS4.Cross();
  if (x_pressed && !last_x_state) {
    digitalWrite(RelayM1_PIN2, !digitalRead(RelayM1_PIN2));
  }
  last_x_state = x_pressed;

  bool L2_pressed = PS4.L2();
  if (L2_pressed && !last_L2_state) {
    digitalWrite(RelayM1_PIN3, !digitalRead(RelayM1_PIN3));
  }
  last_L2_state = L2_pressed;

  bool r2_pressed = PS4.R2();
  if (r2_pressed && !last_r2_state) {
    digitalWrite(RelayM1_PIN4, !digitalRead(RelayM1_PIN4));
  }
  last_r2_state = r2_pressed;

  // sent to slave
  bool square_pressed = PS4.Square();
  if (square_pressed && !last_square_state) {
    Wire.beginTransmission(Address_Small);
    Wire.write('A');
    Wire.endTransmission();
  }
  last_square_state = square_pressed;

  bool triangle_pressed = PS4.Triangle();
  if (triangle_pressed && !last_triangle_state) {
    Wire.beginTransmission(Address_Small);
    Wire.write('B');
    Wire.endTransmission();
  }
  last_triangle_state = triangle_pressed;

  bool share_pressed = PS4.Share();
  if (share_pressed && !last_share_state) {

  }
  last_share_state = share_pressed;
}

void lift_control() {
  int L_Y = PS4.LStickY();
  char current_state;

  if (abs(L_Y) > LStickY_Calib) {
    if (L_Y > 0) {
      current_state = 'U';
    } else {
      current_state = 'D';
    }
  } else {
    current_state = 'S';
  }

  if (current_state != last_lift_state) {
    Wire.beginTransmission(Address_Small);
    Wire.write(current_state);
    Wire.endTransmission();
    
    last_lift_state = current_state;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  setCpuFrequencyMhz(160);
  PS4.begin("08:a6:f7:10:a8:5c");
  Holding.init();
  pinMode(RelayM1_PIN1, OUTPUT);
  pinMode(RelayM1_PIN2, OUTPUT);
  pinMode(RelayM1_PIN3, OUTPUT);
  pinMode(RelayM1_PIN4, OUTPUT);
  
  digitalWrite(RelayM1_PIN1, HIGH);
  digitalWrite(RelayM1_PIN2, HIGH);
  digitalWrite(RelayM1_PIN3, HIGH);
  digitalWrite(RelayM1_PIN4, HIGH);
}

void loop() {
  update_control();
  digital_control();
  lift_control();

  unsigned long now = millis();
  if ((now - prev_control_time) >= (1000 / COMMAND_RATE)) {
    moveBase();
    prev_control_time = now;
  }
}