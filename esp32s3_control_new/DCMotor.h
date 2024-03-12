#pragma once
#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include "esp32-hal-ledc.h"

/** May be adding more specs to the motor? **/
class DCMotor{
private:
    // variables use for motor control
    int mt_D1;          // direction control one
    int mt_D2;          // direction control two
    // int mt_PWM;      // current pwm of the motor
    int ledc_channel;   // pwm channel for the motor
    int mt_resol;       // pwm resolution
    int mt_freq;        // pwm frequency
    int max_pwm;        // max pwm value

public:
    
    // for motor speed sensing
    volatile long motor_pulse_count = 0;

    int mt_HALL_A; // hall sensor phase A
    int mt_HALL_B; // hall sensor phase B

    // constructor and destructor
    DCMotor();
    DCMotor(int D1, int D2, int PWM, int HALL_A, int HALL_B, int channel, int freq, int resol);
    ~DCMotor();

    // member functions
    void setUpMotor();
    void motorCtrl(int pwmInput);
};

#endif