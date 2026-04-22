#include "Arduino.h"
#include "Motor.h"
#include "config.h"
#include "Holding.h"
#include "Kinematics.h"
#include <PS4Controller.h>

//d4:e9:f4:e2:1c:c8

Motor MotorFL(MotorPinFLM1_A, MotorPinFL_B, MAX_RPM);
Motor MotorFR(MotorPinFRM1_A, MotorPinFR_B, MAX_RPM);
Motor MotorRL(MotorPinRLM1_A, MotorPinRL_B, MAX_RPM);
Motor MotorRR(MotorPinRRM1_A, MotorPinRR_B, MAX_RPM);

Kinematics kinematics(Kinematics::MECANUM, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

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

const int LStickX_Calib = 40;
const int LStickY_Calib = 20;
const int RStickX_Calib = 20;

Holding Holding;

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

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
  float a_x = 0;
  float a_y = 0;
  float a_z = 0;

  if (PS4.Up()) {
    d_x = walk_speed;
  } else if (PS4.Down()) {
    d_x = -walk_speed;
  }
  
  if (PS4.Left()) {
    d_y = slide_speed;
  } else if (PS4.Right()) {
    d_y = -slide_speed;
  }

  if (PS4.L1()) {
    d_z = turn_speed;  // L1 = หมุนตัวทวนเข็มนาฬิกา
  } else if (PS4.R1()) {
    d_z = -turn_speed; // R1 = หมุนตัวตามเข็มนาฬิกา
  }

  int L_X = PS4.LStickX();
  int L_Y = PS4.LStickY();
  int R_X = PS4.RStickX();

  if (abs(L_Y) > LStickY_Calib) {
    a_x = ((float)L_Y / 127.0) * walk_speed;
  }
  if (abs(R_X) > RStickX_Calib) {

    a_y = ((float)R_X / 127.0) * -slide_speed;
  }
  if (abs(L_X) > LStickX_Calib) {
    a_z = ((float)L_X / 127.0) * -turn_speed;
  }

  g_req_linear_vel_x = a_x + d_x;
  g_req_linear_vel_y = a_y + d_y;
  g_req_angular_vel_z = a_z + d_z;

  g_req_linear_vel_x = constrain(g_req_linear_vel_x, -walk_speed, walk_speed);
  g_req_linear_vel_y = constrain(g_req_linear_vel_y, -slide_speed, slide_speed);
  g_req_angular_vel_z = constrain(g_req_angular_vel_z, -turn_speed, turn_speed);
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  PS4.begin("d4:e9:f4:e2:1c:c8");
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