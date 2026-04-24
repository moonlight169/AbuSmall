#include <Arduino.h>
#include <Wire.h>
#include "Motor.h"
#include "config.h"

#define Address 0x04

// เพิ่ม volatile ป้องกันบั๊กเวลาตัวแปรถูกเปลี่ยนค่าข้ามฟังก์ชัน
volatile int lift_target = 0;
volatile char last_command = '\0';
volatile bool new_data_received = false;

Motor MotorLift(MotorPinLift_M1_A, MotorPinLift_M1_B, M_LIFT_MAX_RPM);

// ----------------------------------------------------
// ฟังก์ชันนี้จะทำงานอัตโนมัติ (Interrupt) เมื่อ Master ส่งข้อมูลมาให้
// * ห้ามใช้ Serial.print หรือ delay() ในนี้เด็ดขาด *
// ----------------------------------------------------
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char command = Wire.read();
    last_command = command;
    new_data_received = true;

    if (command == 'F') {
      lift_target = 1;
    } else if (command == 'G') {
      lift_target = 0;
    } else if (command == 'A') {
      digitalWrite(RelayM1_PIN5, !digitalRead(RelayM1_PIN5));
    } else if (command == 'B') {
      digitalWrite(RelayM1_PIN6, !digitalRead(RelayM1_PIN6));
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

    pinMode(LimitM1_SW_UP, INPUT_PULLUP);
    pinMode(LimitM1_SW_DOWN, INPUT_PULLUP);

    pinMode(RelayM1_PIN5, OUTPUT);
    pinMode(RelayM1_PIN6, OUTPUT);
}

void loop() {
    if (new_data_received) {
      Serial.print("Received command: ");
      Serial.println(last_command);
      new_data_received = false; // เคลียร์สถานะ
    }

    // คอยเช็ค Limit Switch และควบคุมมอเตอร์ตลอดเวลา
    // lift_control(); 

    delay(20); 
}