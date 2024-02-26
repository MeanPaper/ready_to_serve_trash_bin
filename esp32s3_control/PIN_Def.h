#ifndef PIN_DEF_H
#define PIN_DEF_H


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

// set up all the project pins
void set_project_pins();


#endif