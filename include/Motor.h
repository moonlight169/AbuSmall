#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class Motor {
private:
    int _pinA, _pinB, _maxRPM, _speed;
    ESP32Encoder *_en; 
    int _counts_per_rev;   
    float _rpm;      
    unsigned long prev_update_time_; 
    long prev_encoder_ticks_;  

public:
    Motor(int pinA, int pinB, int maxRPM, ESP32Encoder *en, int counts_per_rev);
    ~Motor();

    void runRPM(int rpm);
    void run(int speed);
    void run();
    void setSpeed(int speed);
    int getSpeed();
    int getRPM();
};

#endif