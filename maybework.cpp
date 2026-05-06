#include "Arduino.h"
#include "Motor.h"
#include "config.h"
#include "Holding.h"
#include "Kinematics.h"
#include <PS4Controller.h>
#include <Wire.h>

#define Address_Small 0x04
#define COMMAND_RATE 50 // 50Hz (20ms per cycle)

// มอเตอร์และ Kinematics
Motor MotorFL(MotorPinFLM1_A, MotorPinFLM1_B, M_MAX_RPM);
Motor MotorFR(MotorPinFRM1_A, MotorPinFRM1_B, M_MAX_RPM);
Motor MotorRL(MotorPinRLM1_A, MotorPinRLM1_B, M_MAX_RPM);
Motor MotorRR(MotorPinRRM1_A, MotorPinRRM1_B, M_MAX_RPM);
Kinematics kinematics(Kinematics::MECANUM, M_MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

// ตัวแปรสถานะความเร็ว
unsigned long prev_control_time = 0;
float g_req_x = 0, g_req_y = 0, g_req_z = 0;   // Target
float curr_x = 0, curr_y = 0, curr_z = 0;      // Current (Smoothed)

// ค่าคงที่ความเร็ว (ปรับจูนที่นี่)
const float f_walkspeed = 1.2, n_walkspeed = 0.5;
const float f_turnspeed = 3.0, n_turnspeed = 2.0;
const float f_slidespeed = 1.8, n_slidespeed = 1.2;
const float ACCEL_LIMIT = 0.15; // ความนุ่มนวล (ยิ่งน้อยยิ่งสมูท)

// สถานะปุ่มและจอย
bool last_states[6] = {false}; // {opt, circ, sq, tri, x, share}
char last_lift_state = 'S';
const int JOY_DEADZONE = 20; 

// --- ฟังก์ชันเสริมเพื่อความเสถียร ---

void sendToSlave(char command) {
  Wire.beginTransmission(Address_Small);
  Wire.write(command);
  if (Wire.endTransmission() != 0) {
    Serial.println("I2C Error: Slave not responding");
  }
}

float applyRamp(float target, float current, float limit) {
  if (target > current) return min(target, current + limit);
  if (target < current) return max(target, current - limit);
  return target;
}

void moveBase() {
  // ทำ Acceleration Ramping เพื่อความนุ่มนวล
  curr_x = applyRamp(g_req_x, curr_x, ACCEL_LIMIT);
  curr_y = applyRamp(g_req_y, curr_y, ACCEL_LIMIT);
  curr_z = applyRamp(g_req_z, curr_z, ACCEL_LIMIT);

  Kinematics::rpm req_rpm = kinematics.getRPM(curr_x, curr_y, curr_z);
  MotorFL.runRPM(req_rpm.motor1);
  MotorFR.runRPM(req_rpm.motor2);
  MotorRL.runRPM(req_rpm.motor3);
  MotorRR.runRPM(req_rpm.motor4);
}

void update_control() {
  if (!PS4.isConnected()) {
    g_req_x = 0; g_req_y = 0; g_req_z = 0;
    return;
  }

  // เลือก Profile ความเร็ว
  float walk = PS4.R2() ? n_walkspeed : f_walkspeed;
  float turn = PS4.R2() ? n_turnspeed : f_turnspeed;
  float slide = PS4.R2() ? n_slidespeed : f_slidespeed;

  float dx = 0, dy = 0, dz = 0;

  // D-Pad Movement
  if (PS4.Up() || PS4.UpRight() || PS4.UpLeft()) dx = walk;
  else if (PS4.Down() || PS4.DownRight() || PS4.DownLeft()) dx = -walk;
  
  if (PS4.Left() || PS4.UpLeft() || PS4.DownLeft()) dy = slide;
  else if (PS4.Right() || PS4.UpRight() || PS4.DownRight()) dy = -slide;

  if (PS4.L1()) dz = turn;
  else if (PS4.R1()) dz = -turn;

  g_req_x = dx; g_req_y = dy; g_req_z = dz;
}

void digital_control() {
  if (!PS4.isConnected()) return;

  // Relay Toggle Logic (ตัวอย่าง Triangle)
  if (PS4.Triangle() && !last_states[3]) {
    digitalWrite(RelayM1_PIN3, !digitalRead(RelayM1_PIN3));
  }
  last_states[3] = PS4.Triangle();

  // Relay Hold Logic (Cross)
  digitalWrite(RelayM1_PIN2, PS4.Cross() ? LOW : HIGH);

  if (PS4.Share() && !last_states[5]) {
    digitalWrite(RelayM1_PIN1, !digitalRead(RelayM1_PIN1));
  }
  last_states[5] = PS4.Share();

  if (PS4.Options() && !last_states[0]) {
    digitalWrite(RelayM1_PIN4, !digitalRead(RelayM1_PIN4));
  }
  last_states[0] = PS4.Options();

  // Slave Commands
  if (PS4.Square() && !last_states[2]) sendToSlave('A');
  last_states[2] = PS4.Square();

  if (PS4.Circle() && !last_states[1]) sendToSlave('B');
  last_states[1] = PS4.Circle();
}

void lift_control() {
  if (!PS4.isConnected()) return;

  int RY = PS4.RStickY();
  int LY = PS4.LStickY();
  char current_state = last_lift_state;
  bool fast = !PS4.R2();

  if (abs(RY) > JOY_DEADZONE) {
    if (RY > 0) current_state = fast ? 'D' : 'd';
    else current_state = fast ? 'U' : 'u';
  } 
  else if (abs(LY) > JOY_DEADZONE) {
    if (LY > 0) current_state = fast ? 'O' : 'o';
    else current_state = fast ? 'C' : 'c';
  } 
  else {
    // หยุดส่งถ้าไม่มีการโยก (กันค้าง)
    if (last_lift_state != 'S' && last_lift_state != 'A' && last_lift_state != 'B') {
      current_state = 'S';
    }
  }

  if (current_state != last_lift_state) {
    sendToSlave(current_state);
    last_lift_state = current_state;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50); // กันค้างถ้า I2C มีปัญหา
  
  setCpuFrequencyMhz(160);
  PS4.begin("08:a6:f7:10:a8:5c");

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
  if (now - prev_control_time >= (1000 / COMMAND_RATE)) {
    moveBase();
    prev_control_time = now;
  }
}