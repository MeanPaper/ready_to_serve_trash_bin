#include "Proj_Setup.h"

// global go here

// PID sample time
int pid_interval = 100; // in milliseconds

// pulse per revolution... for hall encoder
int ppr = 11; 

// gear reduction ratio, the denominator
double reduction_ratio = 56;    

// number of pulse for the shaft to turn one cycle
double shaft_ppr = ppr * reduction_ratio; 

// for time difference
long currentTime = 0;

// hall encoder counts
volatile long motor_one_pulse = 0; // left motor
volatile long motor_two_pulse = 0; // right motor

// --- --- --- close loop --- --- ---
// PID controller parameters
double Kp = 0.05;   // proportional
double Ki = 0.05;   // integral
double Kd = 0;      // differential

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

// first motor control
void channel_one_Ctrl(float pwmInputOne){
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
void channel_two_Ctrl(float pwmInputTwo){
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