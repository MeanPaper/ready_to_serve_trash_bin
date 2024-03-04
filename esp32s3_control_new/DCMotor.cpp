#include "DCMotor.h"

DCMotor::DCMotor(){}

// main motor object constructor
DCMotor::DCMotor(int D1, int D2, int PWM, int HALL_A, int HALL_B, int channel, int freq=200, int resol=8){
    mt_D1 = D1; 
    mt_D2 = D2;
    mt_HALL_A = HALL_A;
    mt_HALL_B = HALL_B;
    // mt_PWM = PWM;
    // ledc_channel = channel;
    mt_resol = resol;
    mt_freq = freq;
    max_pwm = pow(2, resol)-1;
    pinMode(mt_D1, OUTPUT);
    pinMode(mt_D2, OUTPUT);
    pinMode(mt_HALL_A, INPUT_PULLUP);
    pinMode(mt_HALL_B, INPUT_PULLUP);
    ledcSetup(channel, mt_freq, mt_resol);
    ledcAttachPin(PWM, channel);
}

DCMotor::~DCMotor(){}

// motor control logic
void DCMotor::motorCtrl(int pwmInput){
    
    // motor is not moving
    if(pwmInput == 0){
        digitalWrite(mt_D1, LOW);
        digitalWrite(mt_D2, LOW);
        return;
    }

    // determine the direction of rotation
    if(pwmInput > 0){
        digitalWrite(mt_D1, LOW);
        digitalWrite(mt_D2, HIGH);
        // MIN_PWM <= pwmIntA <= MAX_PWM
        ledcWrite(ledc_channel, constrain(pwmInput, 0, max_pwm));
    }
    else{
        digitalWrite(mt_D1, HIGH);
        digitalWrite(mt_D2, LOW);
        ledcWrite(ledc_channel, (-1)*constrain(pwmInput, -max_pwm, 0));
    }
    
}



