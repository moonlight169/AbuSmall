#include "Arduino.h"
#include "Motor.h"
#include "config.h"
#include "Holding.h"
#include "Kinematics.h"
#include <PS4Controller.h>
#include <Wire.h>
#include <nvs_flash.h>

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

float f_walkspeed  = 1.4;
float n_walkspeed  = 0.5;
float f_turnspeed  = 3.2;
float n_turnspeed  = 2.0;
float f_slidespeed = 2.0;
float n_slidespeed = 1.2;

float walkspeed  = n_walkspeed;
float turnspeed  = n_turnspeed;
float slidespeed = n_slidespeed;

bool last_options_state  = false;
bool last_circle_state   = false;
bool last_square_state   = false;
bool last_triangle_state = false;
bool last_x_state        = false;
bool last_share_state    = false;

char last_lift_state = 'S';

// ลด threshold ลง และเพิ่ม hysteresis เพื่อกันการกระตุก
const int STICK_THRESHOLD_ENTER = 80;   // เข้า zone
const int STICK_THRESHOLD_EXIT  = 50;   // ออก zone (hysteresis)
bool r_stick_active = false;
bool l_stick_active = false;

// ส่ง I2C พร้อม retry เพื่อลด packet loss
bool i2c_send(char cmd) {
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(Address_Small);
    Wire.write(cmd);
    uint8_t err = Wire.endTransmission();
    if (err == 0) return true;
    delayMicroseconds(500);
  }
  return false;
}

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(
    g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  MotorFL.runRPM(req_rpm.motor1);
  MotorFR.runRPM(req_rpm.motor2);
  MotorRL.runRPM(req_rpm.motor3);
  MotorRR.runRPM(req_rpm.motor4);
}

void update_control() {
  if (!PS4.isConnected()) {
    g_req_linear_vel_x  = 0;
    g_req_linear_vel_y  = 0;
    g_req_angular_vel_z = 0;
    MotorFL.run(0); MotorFR.run(0);
    MotorRL.run(0); MotorRR.run(0);
    return;
  }

  bool fast_mode = !PS4.R2();
  walkspeed  = fast_mode ? f_walkspeed  : n_walkspeed;
  turnspeed  = fast_mode ? f_turnspeed  : n_turnspeed;
  slidespeed = fast_mode ? f_slidespeed : n_slidespeed;

  float d_x = 0, d_y = 0, d_z = 0;

  if      (PS4.Up()   || PS4.UpRight() || PS4.UpLeft())   d_x =  walkspeed;
  else if (PS4.Down() || PS4.DownRight()|| PS4.DownLeft()) d_x = -walkspeed;

  if      (PS4.Left()  || PS4.UpLeft()  || PS4.DownLeft())  d_y =  slidespeed;
  else if (PS4.Right() || PS4.UpRight() || PS4.DownRight()) d_y = -slidespeed;

  if      (PS4.L1()) d_z =  turnspeed;
  else if (PS4.R1()) d_z = -turnspeed;

  g_req_linear_vel_x  = constrain(d_x, -walkspeed,  walkspeed);
  g_req_linear_vel_y  = constrain(d_y, -slidespeed, slidespeed);
  g_req_angular_vel_z = constrain(d_z, -turnspeed,  turnspeed);
}

void digital_control() {
  bool triangle_pressed = PS4.Triangle();
  if (triangle_pressed && !last_triangle_state)
    digitalWrite(RelayM1_PIN3, !digitalRead(RelayM1_PIN3));
  last_triangle_state = triangle_pressed;

  // Cross: active LOW ตลอดที่กดค้าง
  digitalWrite(RelayM1_PIN2, PS4.Cross() ? LOW : HIGH);

  bool share_pressed = PS4.Share();
  if (share_pressed && !last_share_state)
    digitalWrite(RelayM1_PIN1, !digitalRead(RelayM1_PIN1));
  last_share_state = share_pressed;

  bool options_pressed = PS4.Options();
  if (options_pressed && !last_options_state)
    digitalWrite(RelayM1_PIN4, !digitalRead(RelayM1_PIN4));
  last_options_state = options_pressed;

  bool square_pressed = PS4.Square();
  if (square_pressed && !last_square_state)
    i2c_send('A');
  last_square_state = square_pressed;

  bool circle_pressed = PS4.Circle();
  if (circle_pressed && !last_circle_state)
    i2c_send('B');
  last_circle_state = circle_pressed;
}

void lift_control() {
  int R_Y = PS4.RStickY();
  int L_Y = PS4.LStickY();

  bool fast_mode = (walkspeed == f_walkspeed);
  char current_state = last_lift_state;

  // R stick: lift up/down — ใช้ hysteresis กันการกระตุกรอบขอบ threshold
  int r_enter = r_stick_active ? STICK_THRESHOLD_EXIT : STICK_THRESHOLD_ENTER;
  if (abs(R_Y) > r_enter) {
    r_stick_active = true;
    current_state = (R_Y > 0)
      ? (fast_mode ? 'D' : 'd')
      : (fast_mode ? 'U' : 'u');
  } else {
    r_stick_active = false;
  }

  // L stick: open/close — ใช้ hysteresis เช่นกัน
  int l_enter = l_stick_active ? STICK_THRESHOLD_EXIT : STICK_THRESHOLD_ENTER;
  if (!r_stick_active && abs(L_Y) > l_enter) {
    l_stick_active = true;
    current_state = (L_Y > 0)
      ? (fast_mode ? 'O' : 'o')
      : (fast_mode ? 'C' : 'c');
  } else if (!r_stick_active) {
    l_stick_active = false;
  }

  // คืนสู่ Stop เฉพาะเมื่อ stick กลับมาที่กลาง และ state ก่อนหน้าเป็น motion
  if (!r_stick_active && !l_stick_active) {
    bool was_moving = (last_lift_state == 'U' || last_lift_state == 'u' ||
                       last_lift_state == 'D' || last_lift_state == 'd');
    if (was_moving) current_state = 'S';
  }

  if (current_state != last_lift_state) {
    i2c_send(current_state);
    last_lift_state = current_state;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz — เสถียรกว่า 400kHz สำหรับ long wire
  setCpuFrequencyMhz(240);

  // ลบ nvs_flash_erase() ออก — ไม่ควร erase ทุก boot
  // ถ้า pairing มีปัญหาค่อย erase ครั้งเดียวด้วยตนเอง
  nvs_flash_init();

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
  if ((now - prev_control_time) >= (1000 / COMMAND_RATE)) {
    moveBase();
    prev_control_time = now;
  }
}