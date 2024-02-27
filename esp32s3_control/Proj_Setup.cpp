#include <Arduino.h>
#include "Proj_Setup.h"

void IRAM_ATTR motorOnePulseIRQ(){
    if(digitalRead(MT1_HALL_A)){
        motor_one_pulse++;
    }
    else{
        motor_one_pulse--;
    }
}

void IRAM_ATTR motorTwoPulseIRQ(){
    if(digitalRead(MT2_HALL_A)){
        motor_two_pulse++;
    }
    else{
        motor_two_pulse--;
    }
}

void setupMotorOne(){ 
    pinMode(MT1_D1, OUTPUT);
    pinMode(MT1_D2, OUTPUT);
    // pinMode(); // for pwm
    pinMode(MT1_HALL_A, INPUT_PULLUP);
    pinMode(MT1_HALL_B, INPUT_PULLUP);
    
    // interrupt 
    attachInterrupt(digitalPinToInterrupt(MT1_HALL_B), motorOnePulseIRQ, RISING);
}

void setupMotorTwo(){

    pinMode(MT2_D1, OUTPUT);
    pinMode(MT2_D2, OUTPUT);
    // pinMode(); // for pwm
    pinMode(MT2_HALL_A, INPUT_PULLUP);
    pinMode(MT2_HALL_B, INPUT_PULLUP);
    
    // interrupt
    attachInterrupt(digitalPinToInterrupt(MT2_HALL_B), motorTwoPulseIRQ, RISING);
}

void setupGearMotors(){
    setupMotorOne();
    setupMotorTwo();
}

void setupLinearActuator(){
    pinMode(Ln_Act_D1, OUTPUT);
    pinMode(Ln_Act_D2, OUTPUT);
    // pinMode(); // for pwm
}