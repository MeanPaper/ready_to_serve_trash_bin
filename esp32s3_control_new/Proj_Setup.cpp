#include "Proj_Setup.h"

#define NOT_CONSIDER 1

#if TESTING==1

// testing motor
int ppr = 3;
double reduction_ratio = 235;
double max_speed = 70;         

#else

// modified the following to match the specs from the motors
int ppr = 11;                       // pulse per revolution (hall encoders)
double reduction_ratio = 56;     // gear reduction ratio, the denominator
double max_speed = 150;
// double reduction_ratio = 169;
// double max_speed = 50;          


#endif

// PID sample time (ms)
int pid_interval = 100; 

// number of pulse for the shaft to turn one cycle
double shaft_ppr = ppr * reduction_ratio; 
double speed_constant = 60 * (1000 / pid_interval) / shaft_ppr;
unsigned long currentTime = 0; 
unsigned long pid_compute_time = 0;

// --- --- --- close loop --- --- ---
// PID controller parameters
#if (TESTING)
// float Kp_one = 1;           // proportional
// float Ki_one = 1.5;         // integral
// float Kd_one = 0.0001;      // differential
#else

/* Actual PID global */

// left motor (finalized)
float Kp_one = 0.8;                 // proportional
float Ki_one = 4;                   // integral
float Kd_one = 0.041;               // differential

// working but not that good
// float Kp_one = 1.15;             // proportional
// float Ki_one = 2.235;            // integral
// float Kd_one = 0.00025;          // differential

// right motor (finalized)
float Kp_two = 0.8;                 // proportional
float Ki_two = 4;                   // integral
float Kd_two = 0.041;               // differential

// the working one
// float Kp_two = 0.9;
// float Ki_two = 1.25;
// float Kd_two = 0.0008;

#endif

/* target, actual, and previous speed of both DC motors */
float targetSpeed_one = 0.0;  
float actualSpeed_one = 0.0;   
float targetSpeed_two = 0.0; 
float actualSpeed_two = 0.0;
float prevSpeed_one = 0.0;
float prevSpeed_two = 0.0;

/* PID computation parameter globals */
float previousError_one = 0.0;
float integral_one = 0.0;
float previousError_two = 0.0;
float integral_two = 0.0;

/* PID outputs */
float output_one = 0;
float output_two = 0;
// --- --- --- --- --- --- --- --- ---

// a command flag, only valid in CLOSE, for opening the lid
bool openLid = false; 

/* PID control object for left and right DC motors */
QuickPID MT1_PID(&actualSpeed_one, &output_one, &targetSpeed_one);
QuickPID MT2_PID(&actualSpeed_two, &output_two, &targetSpeed_two);

/* linear actuator info */
LinearActInfo lid;

/* DC motor objects */
DCMotor Motor_one(MT1_D1, MT1_D2, MT1_PWM, MT1_HALL_A, MT1_HALL_B, CHAN_MT_ONE, MT_FREQ, RESOL);
DCMotor Motor_two(MT2_D1, MT2_D2, MT2_PWM, MT2_HALL_A, MT2_HALL_B, CHAN_MT_TWO, MT_FREQ, RESOL);


/** motorOneIRQ and motorTwoIRQ
 * Interrupt service routine for motor one and two; reads the hall sensor signal
 * and increments/decrements the pulse count based on the signal reading
*/
void IRAM_ATTR motorOneIRQ(){ 
    if(digitalRead(Motor_one.mt_HALL_A)){
        Motor_one.motor_pulse_count++;
    }
    else{
        Motor_one.motor_pulse_count--;
    }
}

void IRAM_ATTR motorTwoIRQ(){
    if(digitalRead(Motor_two.mt_HALL_A)){
        Motor_two.motor_pulse_count++;
    }
    else{
        Motor_two.motor_pulse_count--;
    }
}


/** setupMotorInterrupt()
 * Allows motor hall sensor to trigger external interrupts on
 * rising edges of its signals and call the corresponding interrupt service routine.
*/
void setupMotorInterrupt(){
    attachInterrupt(digitalPinToInterrupt(Motor_one.mt_HALL_B), motorOneIRQ, RISING);
    attachInterrupt(digitalPinToInterrupt(Motor_two.mt_HALL_B), motorTwoIRQ, RISING);
}

/** setpuLinearActuator()
 * set up h-bridge control pins and PWM signal pin for linear actuator
 * and initialize the linear actuator state
*/
void setupLinearActuator(){
    pinMode(Ln_Act_D1, OUTPUT);
    pinMode(Ln_Act_D2, OUTPUT);
    ledcSetup(CHAN_LN_ACT, LA_FREQ, RESOL);
    ledcAttachPin(Ln_Act_PWM, CHAN_LN_ACT);
    
    lid.lidState = CLOSE;
    lid.shouldOpen = true;
    lid.currentTime = 0;
}

/** linearActCtrl()
 * control the linear actuator movement based on the PWM signal input
 * @param pwmInputLnAct: the PWM signal duty cycle for the linear actuator
*/
void linearActCtrl(int pwmInputLnAct){
    if(pwmInputLnAct == 0){ // motor not moving
        digitalWrite(Ln_Act_D1, LOW);
        digitalWrite(Ln_Act_D2, LOW);
        return;
    }
    
    if(pwmInputLnAct > 0){ // forward movement
        digitalWrite(Ln_Act_D1, HIGH);
        digitalWrite(Ln_Act_D2, LOW);
        ledcWrite(CHAN_LN_ACT, constrain(pwmInputLnAct, 0, MAX_PWM));
    }
    else{ // backward movement
        digitalWrite(Ln_Act_D1, LOW);
        digitalWrite(Ln_Act_D2, HIGH);
        ledcWrite(CHAN_LN_ACT, -constrain(pwmInputLnAct, -MAX_PWM, 0));
    }
}

/** stopDCMotor()
 * set both DC motors target speed to be 0
*/
void stopDCMotor(){
    targetSpeed_one = 0.0;
    targetSpeed_two = 0.0;
}

/** motorsOff()
 * turn off h-bridge control for both motors
*/
void motorsOff(){
    Motor_one.motorCtrl(0);
    Motor_two.motorCtrl(0);
}


/** setTargetSpeed()
 * set the target speed for each DC motor
 * @param MTOneSpeed: target speed for motor one (left)
 * @param MTTwoSpeed: target speed for motor two (right)
*/
void setTargetSpeed(float MTOneSpeed, float MTTwoSspeed){
    targetSpeed_one = constrain(MTOneSpeed, -max_speed, max_speed);
    targetSpeed_two = constrain(MTTwoSspeed, -max_speed, max_speed);
}

/** QuickPID_Init()
 * initialize the PID control object for both DC motors
 * set the PID control parameters, output limits, sampling rates
*/
void QuickPID_Init(){
    MT1_PID.Reset();
    MT1_PID.SetTunings(Kp_one, Ki_one, Kd_one);
    MT1_PID.SetSampleTimeUs(pid_interval * 1000);
    MT1_PID.SetMode(MT1_PID.Control::automatic);
    MT1_PID.SetOutputLimits((float)(MAX_PWM) * (-1), (float)(MAX_PWM));

    MT2_PID.Reset();
    MT2_PID.SetTunings(Kp_two, Ki_two, Kd_two);
    MT2_PID.SetSampleTimeUs(pid_interval * 1000);
    MT2_PID.SetMode(MT2_PID.Control::automatic);
    MT2_PID.SetOutputLimits((float)(MAX_PWM) * (-1), (float)(MAX_PWM));
}


/** QuickPID_Compute()
 * compute the error between the target and actual speed of both DC motors
 * then compute the PID control output based on the error
*/
void QuickPID_Compute(){

    // this mind need to fixed later, it is ok for now, but not sure
    noInterrupts();
    if(millis() - pid_compute_time >= pid_interval){
        // compute motor two speed, unit = revolutions per min
        prevSpeed_two = (actualSpeed_two > 1 || actualSpeed_two < -1) ? actualSpeed_two : prevSpeed_two;
        actualSpeed_two = (float)(Motor_two.motor_pulse_count) * speed_constant ;
        // Motor_two.motor_pulse_count = 0; 

        // compute motor one speed, unit = revolutions per min
        prevSpeed_one = (actualSpeed_one > 1 || actualSpeed_one < -1) ? actualSpeed_one : prevSpeed_one;
        actualSpeed_one = (float)(Motor_one.motor_pulse_count) * speed_constant;
        
        // Motor_one.motor_pulse_count = 0;
        MT1_PID.Compute();
        MT2_PID.Compute();

        Motor_one.motorCtrl((int)(round(output_one)));
        Motor_two.motorCtrl((int)(round(output_two)));
        // noInterrupts();

        Motor_two.motor_pulse_count = 0; 
        Motor_one.motor_pulse_count = 0;
        pid_compute_time = millis();
    }
    interrupts();

    /* for debugging */
    // Serial.printf("output_two:");
    // Serial.print(output_two);
    // Serial.print(",");
    // Serial.print("Detected_two:");
    // Serial.print(actualSpeed_two > NOT_CONSIDER || actualSpeed_two < -NOT_CONSIDER ? actualSpeed_two: prevSpeed_two);
    // Serial.print(",");
    // Serial.print("Target Two:");
    // Serial.println(targetSpeed_two);
}

/** setLid()
 * set the lid to be open
*/
void setLid(){
    openLid = true;
}

/** checkLid()
 * check the current state of the lid and control the linear actuator accordingly
 * @return lidStateEnum: the current state of the lid
*/
lidStateEnum checkLid(){
    switch(lid.lidState){
        case OPEN:
            openLid = false;
            lid.shouldOpen = false;
            if(millis() - lid.currentTime > LID_TIME){ // if open is long enough
                lid.lidState = TRANS;
                lid.currentTime = millis();
            }
            break;
        case TRANS:
            openLid = false;
            if(millis() - lid.currentTime <= LID_TRANS_TIME){ // we can make trans time longer
                if(lid.shouldOpen){ // determine if is open or closing
                    linearActCtrl(-200);
                }
                else{
                    linearActCtrl(200);
                }
            }
            else{
                linearActCtrl(0); 
                lid.currentTime = millis();
                if(lid.shouldOpen){
                    lid.lidState = OPEN;
                }
                else{
                    lid.lidState = CLOSE;
                }
            }
            break;
        case CLOSE: // lid is in close state
            lid.currentTime = millis();
            lid.shouldOpen = true;
            lid.lidState = CLOSE;
            if(openLid){
                lid.lidState = TRANS;
            }
            break;
        default:
            break;
    }
    return lid.lidState;
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
unsigned long stop_time = 0;

void parseCmd(){
    while (Serial.available() > 0){
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
            
            // -----------------------------------------------------
            if(cmd == 'm'){ // motor: m
                setTargetTicksPerFrame(atoi(argv1),atoi(argv2));
            }
            
            if(cmd == 'l'){ // linear actuator: l
                if(lid.lidState == CLOSE){
                    openLid = true;
                }
            }
            resetCommand();
            // -----------------------------------------------------

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
}


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
        stop_time = millis();
        setTargetSpeed(0,0);
        moving = 0;
    }
    else
    {
        moving = 1;
    }
    setTargetSpeed(left, right);
}

/* Command format: m mt1_speed mt2_speed */
void plotData() {
    parseCmd();
    
    if(!moving && millis() - stop_time > 2000){
        motorsOff();
    }
    else{
        QuickPID_Compute();

        /*********   comment out if not in debug set up ***********/
        /*********   the ":" and "," should not be comment out if in plotting ***********/
        // Serial.printf("output_one:");
        // Serial.print(output_one);
        // Serial.print(",");
        // Serial.print("Detected_one:");
        // Serial.print(actualSpeed_one > NOT_CONSIDER || actualSpeed_one < -NOT_CONSIDER ? actualSpeed_one: prevSpeed_one);
        // Serial.print(",");
        // Serial.print("Target_one:");
        // Serial.println(targetSpeed_one);

        Serial.printf("output_two:");
        Serial.print(output_two);
        Serial.print(",");
        Serial.print("Detected_two:");
        Serial.print(actualSpeed_two > NOT_CONSIDER || actualSpeed_two < -NOT_CONSIDER ? actualSpeed_two: prevSpeed_two);
        Serial.print(",");
        Serial.print("Target Two:");
        Serial.println(targetSpeed_two);
        /******** comment out the code once PID param tuning is done *********/
    }
}

void PID_compute(){
    if(millis() < currentTime){
        return;
    }
    // compute motor two speed, unit = revolutions per min
    actualSpeed_two = (float)((Motor_two.motor_pulse_count / shaft_ppr) * 60 * (1000 / pid_interval));
    Motor_two.motor_pulse_count = 0;

    // compute motor one speed, unit = revolutions per min
    actualSpeed_one = (float)((Motor_one.motor_pulse_count / shaft_ppr) * 60 * (1000 / pid_interval));
    Motor_one.motor_pulse_count = 0;

    // compute error and adjust control
    double error_one = targetSpeed_one - actualSpeed_one;
    integral_one += error_one;
    double derivative_one = error_one - previousError_one;

    double error_two = targetSpeed_two - actualSpeed_two;
    integral_two += error_two;
    double derivative_two = error_two - previousError_two;

    // compute pid output
    output_one = Kp_one * error_one + Ki_one * integral_one + Kd_one * derivative_one;
    output_two = Kp_two * error_two + Ki_two * integral_two + Kd_two * derivative_two;
    
    // windup prevent windup
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
    Motor_one.motorCtrl((int)round(output_one));
    Motor_two.motorCtrl((int)round(output_two));
    // Serial.printf("error_one: %f, integral: %f, output_one: %f\n", error_one, integral_one, output_one);
    
    // update error
    previousError_one = error_one;
    previousError_two = error_two;

    // update time
    currentTime = millis() + pid_interval;
}

#endif


#define MAX_SPEED_MISS_COUNT    10000

int leftSpeedMiss = 0;
int rightSpeedMiss = 0;

/** speedAutoAdjust() 
 * if target speed cannot be reach, adjust the target speed of the DC motors to the actual speed 
 * after a certain time interval
*/
void speedAutoAdjust(){
    if(targetSpeed_one == 0 && targetSpeed_two == 0){
        return;
    }

    // left motor speed miss count
    if(abs(targetSpeed_one - actualSpeed_one) > 1.5){ // +/- 1.5 within the target is acceptable
        leftSpeedMiss++;
    }
    else{
        leftSpeedMiss = 0;
    }

    // right motor speed miss count
    if(abs(targetSpeed_two - actualSpeed_two) > 1.5){ // +/- 1.5 within the target is acceptable
        rightSpeedMiss++;
    }
    else{
        rightSpeedMiss = 0;
    }

    if(leftSpeedMiss > MAX_SPEED_MISS_COUNT){
        targetSpeed_one = actualSpeed_one;
        leftSpeedMiss = 0;
    }
    
    if(rightSpeedMiss > MAX_SPEED_MISS_COUNT){
        targetSpeed_two = actualSpeed_two;
        rightSpeedMiss = 0;
    }
}


// the idea of this...
// the motor might not be able to meet the target speed if there are more loads
// so we need to adjust the target speed to the actual speed after a certain time interval
// so that the PID control would not become out of control

// a good way would be never drive the motor to the max speed
// if the motor is driver at the max speed then mostly something is wrong with the PID
// then we need to reset the PID to correct the system
