#include "Proj_Setup.h"
// #include <PID_v1.h>

/* set to 1 to use the small motor, for code testing */
#if TESTING==1

int ppr = 3;
double reduction_ratio = 235;
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
#if (TESTING)
float Kp = 0.15;       // proportional
float Ki = 0.75;       // integral
float Kd = 0.05;       // differential
#else
/* pending... need to be adjust */
float Kp = 0.05;   // proportional
float Ki = 0.05;   // integral
float Kd = 0;      // differential
#endif

// target speed and current speed
float targetSpeed_one = 0.0;  
float actualSpeed_one = 0.0;   
float targetSpeed_two = 0.0; 
float actualSpeed_two = 0.0;

// PID controller output and variables
float previousError_one = 0.0;
float integral_one = 0.0;
float previousError_two = 0.0;
float integral_two = 0.0;

float output_one = 0;
float output_two = 0;
// --- --- --- --- --- --- --- --- ---

// To be test
// PID MT1_PID(&actualSpeed_one, &output_one, &targetSpeed_one,Kp, Ki, Kd, DIRECT);
QuickPID MT1_PID(&actualSpeed_one, &output_one, &targetSpeed_one);

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
void motor_one_Ctrl(int pwmInputOne){
    // get int value
    // int pwmIntOne = round(pwmInputOne);

    // value == 0 mean stop
    if(pwmInputOne == 0){
        digitalWrite(MT1_D1, LOW);
        digitalWrite(MT1_D2, LOW);
        return;
    }

    // determine the direction of rotation
    if(pwmInputOne > 0){
        digitalWrite(MT1_D1, LOW);
        digitalWrite(MT1_D2, HIGH);
        // MIN_PWM <= pwmIntA <= MAX_PWM
        ledcWrite(CHAN_MT_ONE, constrain(pwmInputOne, 0, MAX_PWM));
    }
    else{
        digitalWrite(MT1_D1, HIGH);
        digitalWrite(MT1_D2, LOW);
        ledcWrite(CHAN_MT_ONE, (-1)*constrain(pwmInputOne, -MAX_PWM, 0));
    }
}

// second motor control
void motor_two_Ctrl(int pwmInputTwo){
    // int pwmIntTwo = round(pwmInputTwo);
    if(pwmInputTwo == 0){
        digitalWrite(MT2_D1, LOW);
        digitalWrite(MT2_D2, LOW);
        return;
    }

    if(pwmInputTwo > 0){
        digitalWrite(MT1_D1, LOW);
        digitalWrite(MT2_D2, HIGH);
        ledcWrite(CHAN_MT_TWO, constrain(pwmInputTwo, 0, MAX_PWM));
    }
    else{
        digitalWrite(MT1_D1, HIGH);
        digitalWrite(MT1_D2, LOW);
        ledcWrite(CHAN_MT_TWO, (-1)*constrain(pwmInputTwo, -MAX_PWM, 0));
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
    output_one = Kp * error_one + Ki * integral_one + Kd * derivative_one;
    output_two = Kp * error_two + Ki * integral_two + Kd * derivative_two;
    
    if(output_one > MAX_PWM || output_one < (-1)*MAX_PWM){
      integral_one -= error_one;
    }

    if(output_two > MAX_PWM || output_two < (-1)*MAX_PWM){
      integral_two -= error_two;
    }

    // constrain output
    output_one = constrain(output_one, (-1)*MAX_PWM, MAX_PWM);
    output_two = constrain(output_two, (-1)*MAX_PWM, MAX_PWM);

    // output PWM signal, control motor speed
    motor_one_Ctrl((int)round(output_one));
    motor_two_Ctrl((int)round(output_two));
    // Serial.printf("error_one: %f, integral: %f, output_one: %f\n", error_one, integral_one, output_one);
    
    // update error
    previousError_one = error_one;
    previousError_two = error_two;


    // Serial.print("RPM_A: ");
    // Serial.print(actualSpeed_one);
    // Serial.printf(" targetSpeed_one: %f", targetSpeed_one);
    // // Serial.print("   RPM_B: ");
    // // Serial.println(actualSpeed_two);
    // Serial.println("--- --- ---");

    currentTime = millis() + pid_interval;
}

void stopDCMotor(){
    targetSpeed_one = 0;
    targetSpeed_two = 0;
}

void setTargetSpeed(double MTOneSpeed, double MTTwoSspeed){
    targetSpeed_one = constrain(MTOneSpeed, -max_speed, max_speed);
    targetSpeed_two = constrain(MTTwoSspeed, -max_speed, max_speed);
}

/********************************* DEBUG **************************************/

#if(DEBUG)
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index_ = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;
bool moving;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0 ; //NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index_ = 0;
}

void setTargetTicksPerFrame(int left, int right)
{
  if (left == 0 && right == 0)
  {
    motor_one_Ctrl(0);
    motor_two_Ctrl(0);
    moving = 0;
  }
  else
  {
    moving = 1;
  }
  targetSpeed_one = left;
  targetSpeed_two = right;
}

/* Command format: m mt1_speed mt2_speed */
void plotData() {
  while (Serial.available() > 0)
  {

    // Read the next character
    chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13){ // '\r'
      if (arg == 1) {
        argv1[index_] = 0 ;//NULL;
      }
      else if (arg == 2) {
        argv2[index_] = 0; //NULL;
      }
      setTargetTicksPerFrame(atoi(argv1),atoi(argv2));
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' '){
      // Step through the arguments
      if (arg == 0) {arg = 1;}
      else if (arg == 1){
        argv1[index_] = NULL;
        arg = 2;
        index_ = 0;
      }
      continue;
    }
    else{
      if (arg == 0){
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1){
        // Subsequent arguments can be more than one character
        argv1[index_] = chr;
        index_++;
      }
      else if (arg == 2){
        argv2[index_] = chr;
        index_++;
      }
    }
  }

  // compute pid periodically
  if(moving==1){
    // PID_compute();
    QuickPID_Compute();

    /*********   comment out if not in debug set up ***********/
    /*********   the ":" and "," should not be comment out if in plotting ***********/

    // for debug purpose: compare the actual speed to target
    Serial.print("Left_Encoder_value:");
    Serial.print(actualSpeed_one);
    Serial.print(",");
    Serial.print("Target_encoder:");
    Serial.println(targetSpeed_one);
    /******** comment out the code once PID param tuning is done *********/
  }
}
#endif

void QuickPID_Init(){
    MT1_PID.SetTunings(Kp, Ki, Kd);
    MT1_PID.SetMode(MT1_PID.Control::automatic);
    MT1_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
}

void QuickPID_Compute(){
    // compute motor one speed, unit = revolutions per min
    actualSpeed_one = (float)((motor_one_pulse / shaft_ppr) * 60 * (1000 / pid_interval));
    motor_one_pulse = 0;
    MT1_PID.Compute();
    motor_one_Ctrl((int)round(output_one));
}