#include <Arduino.h>
#include <Wire.h>
#include "Motor.h"
#include "config.h"
#include <PS4Controller.h>

#define Address 0x04

volatile char last_command = '\0';
volatile bool new_data_received = false;

// เพิ่มตัวแปรสำหรับเก็บสถานะปัจจุบันของมอเตอร์ (S = Stop, U = Up, D = Down)
volatile char motorState = 'S'; 

const int LStickY_Calib = 20;

// ----------------------------------------------------
// ฟังก์ชัน Interrupt เมื่อ Master ส่งข้อมูลมาให้
// ทำหน้าที่แค่อัปเดตคำสั่ง ห้ามสั่งมอเตอร์ค้างในนี้
// ----------------------------------------------------
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char command = Wire.read();
    last_command = command;
    new_data_received = true;

    // ควบคุม Relay (สลับสถานะได้เลย เพราะทำงานครั้งเดียวจบ)
    if (command == 'A') {
      digitalWrite(RelayM1_PIN5, !digitalRead(RelayM1_PIN5));
    } else if (command == 'B') {
      digitalWrite(RelayM1_PIN6, !digitalRead(RelayM1_PIN6));
    } else if (command == 'C') { 
      // รอใส่คำสั่ง
    } 
    // อัปเดตสถานะการเคลื่อนที่ของมอเตอร์
    else if (command == 'U' || command == 'D' || command == 'S') {
      motorState = command;
    }
  }
}

void requestEvent() {
  // หากต้องการส่งสถานะกลับไปให้ Master สามารถเขียน Wire.write() ที่นี่ได้
}

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

    pinMode(limitSWFPin, INPUT_PULLUP);
    pinMode(limitSWBPin, INPUT_PULLUP);

    pinMode(RelayM1_PIN5, OUTPUT);
    pinMode(RelayM1_PIN6, OUTPUT);
}

void loop() {
  if (motorState == 'U') {
    if (digitalRead(limitSWFPin) == HIGH) { 
        digitalWrite(MotorPinLift_M1_A, HIGH); 
        digitalWrite(MotorPinLift_M1_B, LOW);
    } else {
        digitalWrite(MotorPinLift_M1_A, LOW); 
        digitalWrite(MotorPinLift_M1_B, LOW);
        motorState = 'S'; 
    }
  } 
  else if (motorState == 'D') {
    if (digitalRead(limitSWBPin) == HIGH) {
        digitalWrite(MotorPinLift_M1_A, LOW); 
        digitalWrite(MotorPinLift_M1_B, HIGH);
    } else {
        digitalWrite(MotorPinLift_M1_A, LOW); 
        digitalWrite(MotorPinLift_M1_B, LOW);
        motorState = 'S'; 
    }
  } 
  else if (motorState == 'S') {
      // สั่งหยุดการทำงาน
      digitalWrite(MotorPinLift_M1_A, LOW);
      digitalWrite(MotorPinLift_M1_B, LOW);
  }
}