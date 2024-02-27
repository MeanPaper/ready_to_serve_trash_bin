#ifndef _PROJ_SETUP_H_
#define _PROJ_SETUP_H_

#include <Arduino.h>
#include "esp32-hal-ledc.h"

// #include "Proj_Debug.h"

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

#define RESOL   8

// motor channel frequency, this may not work
#define MT_FREQ 2000

// linear actuator frequency
#define LA_FREQ 2000

#define MAX_PWM pow(2, RESOL)-1   // the max duty cycle
#define MIN_PWM MAX_PWM/5         // the min duty cycle, the motor will not move, if the duty cycle is lower

// motor hall encoder interrupt handler
void IRAM_ATTR motorOnePulseIRQ();
void IRAM_ATTR motorTwoPulseIRQ();

// set up gear motor one 
void setupMotorOne();

// set up gear motor two
void setupMotorTwo();

// set up the linear actuator
void setupLinearActuator();

// set up everything
void setupAllMotors();



#endif