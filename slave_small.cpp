#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#define Address 0x04

volatile char motorState = 'S'; 

// --- ตั้งค่า PWM สำหรับ ESP32 ---
const int freq = 5000;      // ความถี่ 5kHz
const int resolution = 8;   // 0-255
const int pwmChannelA = 4;  // เลือกแชนแนล (ไม่ให้ซ้ำกับตัวอื่น)
const int pwmChannelB = 5;

// ค่าความเร็วที่ต้องการ (0-255)
const int speed_fast = 255;
const int speed_slow = 45; 

void receiveEvent(int howMany) {
  while (Wire.available()) {
    char command = Wire.read();
    if (command == 'A') {
      digitalWrite(RelayM1_PIN5, !digitalRead(RelayM1_PIN5));
    } else if (command == 'B') {
      digitalWrite(RelayM1_PIN6, !digitalRead(RelayM1_PIN6));
    } 
    // รับคำสั่ง U, u, D, d, S
    else if (command == 'U' || command == 'u' || command == 'D' || command == 'd' || command == 'O' || command == 'o' || command == 'C' || command == 'c' || command == 'S') {
      motorState = command;
    }
  }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(Address); 
    Wire.onReceive(receiveEvent); 

    // เริ่มต้นระบบ PWM ของ ESP32
    ledcSetup(pwmChannelA, freq, resolution);
    ledcSetup(pwmChannelB, freq, resolution);
    ledcAttachPin(MotorPinLift_M1_A, pwmChannelA);
    ledcAttachPin(MotorPinLift_M1_B, pwmChannelB);

    pinMode(limitSWFPin, INPUT_PULLUP);
    pinMode(limitSWBPin, INPUT_PULLUP);
    pinMode(RelayM1_PIN5, OUTPUT);
    pinMode(RelayM1_PIN6, OUTPUT);

    digitalWrite(RelayM1_PIN5, HIGH);
    digitalWrite(RelayM1_PIN6, HIGH);
}

void stopLift() {
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);
}

void loop() {
  bool hit_top = (digitalRead(limitSWFPin) == LOW);
  bool hit_bottom = (digitalRead(limitSWBPin) == LOW);

  // 2. จัดการสถานะมอเตอร์
  if (motorState == 'U' || motorState == 'u' || motorState == 'O' || motorState == 'o') {
    // โหมดหมุนขึ้น (ทั้ง Manual และ Auto)
    if (!hit_top) {
      int speed = (motorState == 'U' || motorState == 'O') ? speed_fast : speed_slow;
      ledcWrite(pwmChannelA, speed);
      ledcWrite(pwmChannelB, 0);
    } else {
      stopLift(); // ชนแล้วหยุด และเซต motorState = 'S'
    }
  } 
  else if (motorState == 'D' || motorState == 'd' || motorState == 'C' || motorState == 'c') {
    // โหมดหมุนลง (ทั้ง Manual และ Auto)
    if (!hit_bottom) {
      int speed = (motorState == 'D' || motorState == 'C') ? speed_fast : speed_slow;
      ledcWrite(pwmChannelA, 0);
      ledcWrite(pwmChannelB, speed);
    } else {
      stopLift(); // ชนแล้วหยุด และเซต motorState = 'S'
    }
  } 
  else {
    stopLift();
  }
}