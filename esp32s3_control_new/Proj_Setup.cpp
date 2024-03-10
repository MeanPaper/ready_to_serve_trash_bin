#include "Proj_Setup.h"

/* set to 1 to use the small motor, for code testing */
#if TESTING==1

int ppr = 3;
double reduction_ratio = 235;
double max_speed = 70; // this is the rpm

#else

int ppr = 11; // pulse per revolution... for hall encoder
double reduction_ratio = 56;  // gear reduction ratio, the denominator
double max_speed = 150;

#endif

// PID sample time (ms)
int pid_interval = 100; 

// number of pulse for the shaft to turn one cycle
double shaft_ppr = ppr * reduction_ratio; 

double speed_constant = 60 * (1000 / pid_interval) / shaft_ppr;
// for time difference
unsigned long currentTime = 0;

// --- --- --- close loop --- --- ---
// PID controller parameters
#if (TESTING)
// float Kp_one = 1;           // proportional
// float Ki_one = 1.5;         // integral
// float Kd_one = 0.0001;      // differential
#else

float Kp_one = 0.9;        // proportional
float Ki_one = 1.25;           // integral
float Kd_one = 0.0008;      // differential

float Kp_two = 0.9;        // proportional
float Ki_two = 1.25;           // integral
float Kd_two = 0.0008;      // differential
#endif

// target speed and current speed
float targetSpeed_one = 0.0;  
float actualSpeed_one = 0.0;   
float targetSpeed_two = 0.0; 
float actualSpeed_two = 0.0;
float prevSpeed_one = 0.0;
float prevSpeed_two = 0.0;

// PID controller output and variables
float previousError_one = 0.0;
float integral_one = 0.0;
float previousError_two = 0.0;
float integral_two = 0.0;

float output_one = 0;
float output_two = 0;
// --- --- --- --- --- --- --- --- ---

// a command flag, only valid in CLOSE, for opening the lid
bool openLid = false; 
bool quick_pid_update;

// To be test
QuickPID MT1_PID(&actualSpeed_one, &output_one, &targetSpeed_one);
QuickPID MT2_PID(&actualSpeed_two, &output_two, &targetSpeed_two);

LinearActInfo lid;
DCMotor Motor_one(MT1_D1, MT1_D2, MT1_PWM, MT1_HALL_A, MT1_HALL_B, CHAN_MT_ONE, MT_FREQ, RESOL);
DCMotor Motor_two(MT2_D1, MT2_D2, MT2_PWM, MT2_HALL_A, MT2_HALL_B, CHAN_MT_TWO, MT_FREQ, RESOL);

// DC motor hall encoder interrupt handler
void IRAM_ATTR motorOneIRQ(){ 
    if(digitalRead(Motor_one.mt_HALL_A)){
        // if(Motor_one.cw)
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

// attach interrupt handler to the Motors
void setupMotorInterrupt(){
    attachInterrupt(digitalPinToInterrupt(Motor_one.mt_HALL_B), motorOneIRQ, RISING);
    attachInterrupt(digitalPinToInterrupt(Motor_two.mt_HALL_B), motorTwoIRQ, RISING);
}

void setupLinearActuator(){
    pinMode(Ln_Act_D1, OUTPUT);
    pinMode(Ln_Act_D2, OUTPUT);
    ledcSetup(CHAN_LN_ACT, LA_FREQ, RESOL);
    ledcAttachPin(Ln_Act_PWM, CHAN_LN_ACT);
    
    // maybe there is a way to make it better
    lid.lidState = CLOSE;
    lid.shouldOpen = true;
    lid.currentTime = 0;
}

void linearActCtrl(int pwmInputLnAct){
    if(pwmInputLnAct == 0){ // motor not moving
        digitalWrite(Ln_Act_D1, LOW);
        digitalWrite(Ln_Act_D2, LOW);
        return;
    }
    
    // TODO: need to test
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

// update output values
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

void stopDCMotor(){
    targetSpeed_one = 0.0;
    targetSpeed_two = 0.0;
}

void setTargetSpeed(float MTOneSpeed, float MTTwoSspeed){
    // Motor_one.ccw = MTOneSpeed >= 0;
    // Motor_two.ccw = MTOneSpeed >= 0;
    // targetSpeed_one = abs(constrain(MTOneSpeed, -max_speed, max_speed));
    // targetSpeed_two = abs(constrain(MTTwoSspeed, -max_speed, max_speed));
    targetSpeed_one = constrain(MTOneSpeed, -max_speed, max_speed);
    targetSpeed_two = constrain(MTTwoSspeed, -max_speed, max_speed);
}

void QuickPID_Init(){
    MT1_PID.SetTunings(Kp_one, Ki_one, Kd_one);
    MT1_PID.SetSampleTimeUs(pid_interval * 1000);
    MT1_PID.SetMode(MT1_PID.Control::automatic);
    MT1_PID.SetOutputLimits((float)(MAX_PWM) * (-1), (float)(MAX_PWM));

    MT2_PID.SetTunings(Kp_two, Ki_two, Kd_two);
    MT2_PID.SetSampleTimeUs(pid_interval * 1000);
    MT2_PID.SetMode(MT2_PID.Control::automatic);
    MT2_PID.SetOutputLimits((float)(MAX_PWM) * (-1), (float)(MAX_PWM));
}

// this mind need to fixed later, it is ok for now
void QuickPID_Compute(){

    // ------ critical section begin -------
    // compute motor two speed, unit = revolutions per min
    noInterrupts();
    prevSpeed_two = (actualSpeed_two > 1 || actualSpeed_two < -1) ? actualSpeed_two : prevSpeed_two;
    // actualSpeed_two = (float)((abs(Motor_two.motor_pulse_count) / shaft_ppr) * 60 * (1000 / pid_interval));
    actualSpeed_two = (float)(Motor_two.motor_pulse_count) * speed_constant ;
    Motor_two.motor_pulse_count = 0; 
    // compute motor one speed, unit = revolutions per min
    prevSpeed_one = (actualSpeed_one > 1 || actualSpeed_one < -1) ? actualSpeed_one : prevSpeed_one;
    // actualSpeed_one = (float)((abs(Motor_one.motor_pulse_count) / shaft_ppr) * 60 * (1000 / pid_interval));
    actualSpeed_one = (float)(Motor_one.motor_pulse_count) * speed_constant;
    Motor_one.motor_pulse_count = 0;
    interrupts();
    // ------ critical section end -------
    
    MT1_PID.Compute();
    MT2_PID.Compute();

    Motor_one.motorCtrl((int)(round(output_one)));
    Motor_two.motorCtrl((int)(round(output_two)));
}

void checkLid(){
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
}

/********************************* DEBUG **************************************/

#if(DEBUG)
// A pair of varibles to help parse serial commands (thanks Fergs)
#define NOT_CONSIDER 1
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
    
    if(!moving && millis() - stop_time > 1000){
        Motor_one.motorCtrl(0);
        Motor_two.motorCtrl(0);
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
        Serial.print("Target_one:");
        Serial.println(targetSpeed_two);
        /******** comment out the code once PID param tuning is done *********/
    }
}
#endif
