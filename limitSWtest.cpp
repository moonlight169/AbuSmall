#include <Arduino.h>
#define limitSWFPin 27
#define limitSWBPin 13

void setup() {
    Serial.begin(115200);
    pinMode(limitSWFPin, INPUT_PULLUP);
    pinMode(limitSWBPin, INPUT_PULLUP);
}

void loop() {
    Serial.print("Front: ");
    Serial.println(digitalRead(limitSWFPin));
    Serial.print("Back: ");
    Serial.println(digitalRead(limitSWBPin));
    delay(1000);
}