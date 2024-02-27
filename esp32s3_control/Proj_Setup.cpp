#include "Proj_Setup.h"

volatile long motor_one_pulse = 0; // left motor
volatile long motor_two_pulse = 0; // right motor

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
    ledcSetup(CHAN_MT_ONE, MT_FREQ, RESOL);
    ledcAttachPin(MT1_PWM, CHAN_MT_ONE);
    
    // interrupt 
    attachInterrupt(digitalPinToInterrupt(MT1_HALL_B), motorOnePulseIRQ, RISING);
}

void setupMotorTwo(){

    pinMode(MT2_D1, OUTPUT);
    pinMode(MT2_D2, OUTPUT);
    pinMode(MT2_HALL_A, INPUT_PULLUP);
    pinMode(MT2_HALL_B, INPUT_PULLUP);
    // pinMode(); // for pwm
    ledcSetup(CHAN_MT_TWO, MT_FREQ, RESOL);
    ledcAttachPin(MT2_PWM, CHAN_MT_TWO);

    // interrupt
    attachInterrupt(digitalPinToInterrupt(MT2_HALL_B), motorTwoPulseIRQ, RISING);
}

void setupLinearActuator(){
    pinMode(Ln_Act_D1, OUTPUT);
    pinMode(Ln_Act_D2, OUTPUT);
    // pinMode(); // for pwm
    ledcSetup(CHAN_LN_ACT, LA_FREQ, RESOL);
    ledcAttachPin(CHAN_LN_ACT, CHAN_LN_ACT);
}

void setupAllMotors(){
    setupMotorOne();
    setupMotorTwo();
    setupLinearActuator();
}