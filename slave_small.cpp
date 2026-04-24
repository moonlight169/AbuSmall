#include <Arduino.h>
#include <Wire.h>
#include "Motor.h"
#include "config.h"

#define Address 0x04
int lift_target = 0; // 0 = ลง, 1 = ขึ้น

Motor MotorLift(MotorPinLift_M1_A, MotorPinLift_M1_B, M_LIFT_MAX_RPM);

// ----------------------------------------------------
// ฟังก์ชันนี้จะทำงานอัตโนมัติ (Interrupt) เมื่อ Master ส่งข้อมูลมาให้
// ----------------------------------------------------
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char command = Wire.read();
    if (command == 'F') {
      lift_target = 1;
      Serial.println("Received command: F (Lift Up)");
    } else if (command == 'G') {
      lift_target = 0;
      Serial.println("Received command: G (Lift Down)");

    } else if(command == 'A') {
      Serial.println("Received command: A");
    } else if(command == 'B') {
      Serial.println("Received command: B");
    } else if(command == 'C') {
      Serial.println("Received command: C");
    } else if(command == 'D') {
      Serial.println("Received command: D");
    } else if(command == 'E') {
      Serial.println("Received command: E");
    } else {
      Serial.print("Received unknown command: ");
      Serial.println(command);
    }
  }
}

// ----------------------------------------------------
// ฟังก์ชันนี้จะทำงานอัตโนมัติ เมื่อ Master ร้องขอข้อมูล (ถ้ามี)
// ----------------------------------------------------
void requestEvent() {
  // หากต้องการส่งสถานะกลับไปให้ Master สามารถเขียน Wire.write() ที่นี่ได้
}

// ----------------------------------------------------
// ฟังก์ชันควบคุมมอเตอร์และเช็ค Limit Switch (ทำงานตลอดเวลา)
// ----------------------------------------------------
// void lift_control() {
//   // อ่านค่า Limit Switch ตลอดเวลา
//   bool is_at_top = (digitalRead(LimitM1_SW_UP) == LOW);
//   bool is_at_bottom = (digitalRead(LimitM1_SW_DOWN) == LOW);

//   if (lift_target == 1) { 
//     // คำสั่งให้ขึ้น
//     if (!is_at_top) {
//       MotorLift.runRPM(170); // ยังไม่ชนลิมิตบน ให้หมุนขึ้นต่อไป
//     } else {
//       MotorLift.runRPM(0);   // ชนลิมิตบนแล้ว สั่งหยุดมอเตอร์ทันที
//     }
//   } else {
//     // คำสั่งให้ลง
//     if (!is_at_bottom) {
//       MotorLift.runRPM(-170); // ยังไม่ชนลิมิตล่าง ให้หมุนลงต่อไป
//     } else {
//       MotorLift.runRPM(0);    // ชนลิมิตล่างแล้ว สั่งหยุดมอเตอร์ทันที
//     }
//   }
// }

void setup() {
    Serial.begin(115200);

    Wire.begin(Address); 

    // ลงทะเบียน Event สำหรับ I2C เพื่อให้บอร์ดรู้ว่าต้องเรียกฟังก์ชันไหน
    Wire.onReceive(receiveEvent); 
    Wire.onRequest(requestEvent);

    pinMode(LimitM1_SW_UP, INPUT_PULLUP);
    pinMode(LimitM1_SW_DOWN, INPUT_PULLUP);
}

void loop() {
    // คอยเช็ค Limit Switch และควบคุมมอเตอร์ตลอดเวลา
    // lift_control(); 

    delay(20); 
}