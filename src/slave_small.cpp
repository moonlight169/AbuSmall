#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#define Address 0x04

// volatile สำหรับตัวแปรที่ ISR เขียน
volatile char _motorState = 'S';

const int freq       = 5000;
const int resolution = 8;
const int pwmChannelA = 4;
const int pwmChannelB = 5;

const int speed_fast = 150;
const int speed_slow = 45;

// debounce time สำหรับลิมิตสวิตช์ (ms)
const unsigned long DEBOUNCE_MS = 10;
unsigned long lastDebounceTop    = 0;
unsigned long lastDebounceBottom = 0;
bool stableTop    = false;
bool stableBottom = false;

// อ่าน motorState อย่างปลอดภัยจาก ISR
char getMotorState() {
  noInterrupts();
  char s = _motorState;
  interrupts();
  return s;
}

void setMotorState(char s) {
  noInterrupts();
  _motorState = s;
  interrupts();
}

// ISR — สั้นที่สุดเท่าที่ทำได้ ห้ามมี delay / Serial / Wire call
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char command = Wire.read();
    if (command == 'A') {
      digitalWrite(RelayM1_PIN5, !digitalRead(RelayM1_PIN5));
    } else if (command == 'B') {
      digitalWrite(RelayM1_PIN6, !digitalRead(RelayM1_PIN6));
    } else if (command == 'U' || command == 'u' ||
               command == 'D' || command == 'd' ||
               command == 'O' || command == 'o' ||
               command == 'C' || command == 'c' ||
               command == 'S') {
      _motorState = command;  // ปลอดภัย — char 1 byte atomic บน ESP32
    }
  }
}

void stopLift() {
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0); 
}

// อ่านสวิตช์พร้อม debounce
void updateLimitSwitches() {
  unsigned long now = millis();

  bool rawTop = (digitalRead(limitSWFPin) == LOW);
  if (rawTop != stableTop) {
    if (now - lastDebounceTop > DEBOUNCE_MS) {
      stableTop = rawTop;
    }
    lastDebounceTop = now;
  }

  bool rawBottom = (digitalRead(limitSWBPin) == LOW);
  if (rawBottom != stableBottom) {
    if (now - lastDebounceBottom > DEBOUNCE_MS) {
      stableBottom = rawBottom;
    }
    lastDebounceBottom = now;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(Address);
  Wire.onReceive(receiveEvent);

  // ESP32 Arduino core v2.x
  ledcSetup(pwmChannelA, freq, resolution);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(MotorPinLift_M1_A, pwmChannelA);
  ledcAttachPin(MotorPinLift_M1_B, pwmChannelB);

  pinMode(limitSWFPin,  INPUT_PULLUP);
  pinMode(limitSWBPin,  INPUT_PULLUP);
  pinMode(RelayM1_PIN5, OUTPUT);
  pinMode(RelayM1_PIN6, OUTPUT);

  digitalWrite(RelayM1_PIN5, HIGH);
  digitalWrite(RelayM1_PIN6, HIGH);

  stopLift();
}

void loop() {
  updateLimitSwitches();

  char state = getMotorState();

  bool goingUp   = (state == 'U' || state == 'u' || state == 'O' || state == 'o');
  bool goingDown = (state == 'D' || state == 'd' || state == 'C' || state == 'c');

  if (goingUp) {
    if (stableTop) {
      // ชนลิมิตบน — หยุดและล็อค state ไม่ให้วนซ้ำ
      stopLift();
      setMotorState('S');
    } else {
      int speed = (state == 'U' || state == 'O') ? speed_fast : speed_slow;
      ledcWrite(pwmChannelA, speed);
      ledcWrite(pwmChannelB, 0);
    }
  } else if (goingDown) {
    if (stableBottom) {
      // ชนลิมิตล่าง — หยุดและล็อค state
      stopLift();
      setMotorState('S');
    } else {
      int speed = (state == 'D' || state == 'C') ? speed_fast : speed_slow;
      ledcWrite(pwmChannelA, 0);
      ledcWrite(pwmChannelB, speed);
    }
  } else {
    stopLift();
  }
}