#include <Arduino.h>
#define limitSWPin 5

void setup() {
    Serial.begin(115200);
    pinMode(limitSWPin, INPUT_PULLUP);
}

void loop() {
    Serial.println(digitalRead(limitSWPin));
    delay(1000);
}