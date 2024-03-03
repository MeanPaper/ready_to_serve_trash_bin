#include "Proj_Setup.h"

/* set to 1 to use the small motor, for code testing */
#define TESTING 1

#if TESTING==1

int ppr = 3;
double reduction_ratio = 200;
double max_speed = 70; // this is the rpm

#else

// pulse per revolution... for hall encoder
int ppr = 11; 
// gear reduction ratio, the denominator
double reduction_ratio = 56;   
double max_speed = 150;

#endif

// PID sample time
int pid_interval = 100; // in milliseconds

// number of pulse for the shaft to turn one cycle
double shaft_ppr = ppr * reduction_ratio; 

// for time difference
long currentTime = 0;

// hall encoder counts
volatile long motor_one_pulse = 0; // left motor
volatile long motor_two_pulse = 0; // right motor

// --- --- --- close loop --- --- ---
// PID controller parameters
double Kp = 15;   // proportional
double Ki = 30;   // integral
double Kd = 0.0001;      // differential

// target speed and current speed
double targetSpeed_one = 100.0;  
double actualSpeed_one = 0.0;   
double targetSpeed_two = 100.0; 
double actualSpeed_two = 0.0;

// PID controller output and variables
double previousError_one = 0.0;
double integral_one = 0.0;
double previousError_two = 0.0;
double integral_two = 0.0;
// --- --- --- --- --- --- --- --- ---


/* function store in mem */
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

/* function store in flask */
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
    targetSpeed_one = 0;
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
    targetSpeed_two = 0;
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

// first motor control
void motor_one_Ctrl(float pwmInputOne){
    // get int value
    int pwmIntOne = round(pwmInputOne);

    // value == 0 mean stop
    if(pwmIntOne == 0){
        digitalWrite(MT1_D1, LOW);
        digitalWrite(MT1_D2, LOW);
        return;
    }

    // determine the direction of rotation
    if(pwmIntOne > 0){
        digitalWrite(MT1_D1, LOW);
        digitalWrite(MT1_D2, HIGH);
        // MIN_PWM <= pwmIntA <= MAX_PWM
        ledcWrite(CHAN_MT_ONE, constrain(pwmIntOne, MIN_PWM, MAX_PWM));
    }
    else{
        digitalWrite(MT1_D1, HIGH);
        digitalWrite(MT1_D2, LOW);
        ledcWrite(CHAN_MT_ONE, -constrain(pwmIntOne, -MAX_PWM, 0));
    }
}

// second motor control
void motor_two_Ctrl(float pwmInputTwo){
    int pwmIntTwo = round(pwmInputTwo);
    if(pwmIntTwo == 0){
        digitalWrite(MT2_D1, LOW);
        digitalWrite(MT2_D2, LOW);
        return;
    }

    if(pwmIntTwo > 0){
        digitalWrite(MT1_D1, LOW);
        digitalWrite(MT2_D2, HIGH);
        ledcWrite(CHAN_MT_TWO, constrain(pwmIntTwo, 0, MAX_PWM));
    }
    else{
        digitalWrite(MT1_D1, HIGH);
        digitalWrite(MT1_D2, LOW);
        ledcWrite(CHAN_MT_TWO,-constrain(pwmIntTwo, -MAX_PWM, 0));
    }
}

// update output values
void PID_compute(){
    if(millis() < currentTime){
        return;
    }
    // compute motor two speed, unit = revolutions per min
    actualSpeed_two = (float)((motor_two_pulse / shaft_ppr) * 60 * (1000 / pid_interval));
    motor_two_pulse = 0;

    // compute motor one speed, unit = revolutions per min
    actualSpeed_one = (float)((motor_one_pulse / shaft_ppr) * 60 * (1000 / pid_interval));
    motor_one_pulse = 0;

    // compute error and adjust control
    double error_one = targetSpeed_one - actualSpeed_one;
    integral_one += error_one;
    double derivative_one = error_one - previousError_one;

    double error_two = targetSpeed_two - actualSpeed_two;
    integral_two += error_two;
    double derivative_two = error_two - previousError_two;

    // compute pid output
    double output_one = Kp * error_one + Ki * integral_one + Kd * derivative_one;
    double output_two = Kp * error_two + Ki * integral_two + Kd * derivative_two;
    
    // constrain output
    output_one = constrain(output_one, (-1)*MAX_PWM, MAX_PWM);
    output_two = constrain(output_two, (-1)*MAX_PWM, MAX_PWM);

    // output PWM signal, control motor speed
    motor_one_Ctrl(output_one);
    motor_two_Ctrl(output_two);
    Serial.printf("error_one: %f, integral: %f, output_one: %f\n", error_one, integral_one, output_one);
    
    // update error
    previousError_one = error_one;
    previousError_two = error_two;

    Serial.print("RPM_A: ");
    Serial.print(actualSpeed_one);
    // Serial.printf(" targetSpeed_one: %f", targetSpeed_one);
    // // Serial.print("   RPM_B: ");
    // // Serial.println(actualSpeed_two);
    Serial.println("--- --- ---");

    // delay(pid_interval);
    currentTime = millis() + pid_interval;
}

void stopDCMotor(){
    targetSpeed_one = 0;
    targetSpeed_two = 0;
}

void setSpeed(double MTOneSpeed, double MTTwoSspeed){
    targetSpeed_one = constrain(MTOneSpeed, -max_speed, max_speed);
    targetSpeed_two = constrain(MTTwoSspeed, -max_speed, max_speed);
}