#pragma once
#ifndef PROJ_SETUP_H
#define PROJ_SETUP_H

#include <Arduino.h>
#include <QuickPID.h>
#include "DCMotor.h"

#define TESTING 1
#define DEBUG 1

/**
  * DC Motor Control 
  * MT# = motor #
  * D1 and D2 are used for H-bridges control
  * D1 (out) - motor direction control 1
  * D2 (out) - motor direction control 2
  *
  * HALL_A and HALL_B are the pins for motor hall sensors, 
  * used for determining motor speed and direction
  * HALL_A (in) - hall sensor phase A
  * HALL_B (in) - hall snstor phase B
  *
  * PWM is the PWM signal for the motor driver, voltage/speed control 
  * PWM (out) - output 
  *
  */

// motor 1 control pins
#define MT1_D1      11
#define MT1_D2      12
#define MT1_PWM     13
#define MT1_HALL_A  16
#define MT1_HALL_B  18

// motor 2 control pins
#define MT2_D1      38
#define MT2_D2      35
#define MT2_PWM     14
#define MT2_HALL_A  17
#define MT2_HALL_B  21

/**
  * Linear Actuator
  * D1 and D2 are used for H-bridges control
  * D1 (out) - motor direction control 1
  * D2 (out) - motor direction control 2
  *
  * PWM is the PWM signal for the motor driver, voltage/speed control 
  * PWM (out) - output 
  *
  */
// linear actuator pins 
#define Ln_Act_D1   36
#define Ln_Act_D2   37
#define Ln_Act_PWM  15

/* PWM control section */
// channels
#define CHAN_MT_ONE 1
#define CHAN_MT_TWO 2
#define CHAN_LN_ACT 3

#define RESOL   8                 // resolution of the PWM signal
#define MT_FREQ 200               // motor channel frequency, this may not work
#define LA_FREQ 200               // linear actuator frequency
#define MAX_PWM pow(2, RESOL)-1   // the max duty cycle
#define LID_TIME 10000            // the lid will open for 10 sec, for testing purpose
#define LID_TRANS_TIME 6500       // the lid transition time will be 6 sec

enum lidStateEnum {OPEN=0, TRANS=1, CLOSE=2}; // the state of the lid

typedef struct LinearActInfo{
    lidStateEnum lidState;
    bool shouldOpen;
    unsigned long currentTime;
} LinearActInfo;


// set up interrupt for the motors
void setupMotorInterrupt();

// set up the linear actuator
void setupLinearActuator();
void linearActCtrl(int pwmInputLnAct); // need to figure this out

// compute PID
void PID_compute();

// stop dc motors
void stopDCMotor();

// set speed
void setTargetSpeed(float MTOneSpeed, float MTTwoSspeed);

void QuickPID_Init();
void QuickPID_Compute();

void checkLid();

// for debug
void plotData();
void parseCmd();
void resetCommand();
void setTargetTicksPerFrame(int left, int right);


// ------ for testing ------

#endif