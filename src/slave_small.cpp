#include <Arduino.h>
#include <Wire.h>
#include "Motor.h"
#include "config.h"
#include <PS4Controller.h>

#define Address 0x04

// เพิ่ม volatile ป้องกันบั๊กเวลาตัวแปรถูกเปลี่ยนค่าข้ามฟังก์ชัน
volatile char last_command = '\0';
volatile bool new_data_received = false;

const int LStickY_Calib = 20;

// ----------------------------------------------------
// ฟังก์ชันนี้จะทำงานอัตโนมัติ (Interrupt) เมื่อ Master ส่งข้อมูลมาให้
// * ห้ามใช้ Serial.print หรือ delay() ในนี้เด็ดขาด *
// ----------------------------------------------------
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char command = Wire.read();
    last_command = command;
    new_data_received = true;

    if (command == 'A') {
      digitalWrite(RelayM1_PIN5, !digitalRead(RelayM1_PIN5));
    } else if (command == 'B') {
      digitalWrite(RelayM1_PIN6, !digitalRead(RelayM1_PIN6));
    } else if (command == 'C') { 
      // รอใส่คำสั่ง
    } else if (command == 'U'){
      digitalWrite(MotorPinLift_M1_A, HIGH); 
      digitalWrite(MotorPinLift_M1_B, LOW);
    } else if (command == 'D') {
      digitalWrite(MotorPinLift_M1_A, LOW);
      digitalWrite(MotorPinLift_M1_B, HIGH);
    } else if (command == 'S') {
      digitalWrite(MotorPinLift_M1_A, LOW);
      digitalWrite(MotorPinLift_M1_B, LOW);
    }
  }
}

void requestEvent() {
  // หากต้องการส่งสถานะกลับไปให้ Master สามารถเขียน Wire.write() ที่นี่ได้
}

// ----------------------------------------------------
// ฟังก์ชันควบคุมมอเตอร์และเช็ค Limit Switch
// ----------------------------------------------------
// void lift_control() {
//   bool is_at_top = (digitalRead(LimitM1_SW_UP) == LOW);
//   bool is_at_bottom = (digitalRead(LimitM1_SW_DOWN) == LOW);

//   if (lift_target == 1) { 
//     if (!is_at_top) {
//       MotorLift.runRPM(170);
//     } else {
//       MotorLift.runRPM(0);  
//     }
//   } else {
//     if (!is_at_bottom) {
//       MotorLift.runRPM(-170);
//     } else {
//       MotorLift.runRPM(0);    
//     }
//   }
// }

void setup() {
    Serial.begin(115200);
    Wire.begin(Address); 

    Wire.onReceive(receiveEvent); 
    Wire.onRequest(requestEvent);

    pinMode(MotorPinLift_M1_A, OUTPUT);
    pinMode(MotorPinLift_M1_B, OUTPUT);
    
    digitalWrite(MotorPinLift_M1_A, LOW);
    digitalWrite(MotorPinLift_M1_B, LOW);

    PS4.begin("d4:e9:f4:e2:5e:44");

    pinMode(LimitM1_SW_UP, INPUT_PULLUP);
    pinMode(LimitM1_SW_DOWN, INPUT_PULLUP);

    pinMode(RelayM1_PIN5, OUTPUT);
    pinMode(RelayM1_PIN6, OUTPUT);
}

void loop() {
    // คอยเช็ค Limit Switch และควบคุมมอเตอร์ตลอดเวลา
    // lift_control(); 

    delay(20); 
}