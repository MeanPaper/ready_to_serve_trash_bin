#include <Arduino.h>
#include "Proj_Debug.h"

void debug_pins_basic(){
  
  // motor1
  pinMode(MT1_D1, OUTPUT);
  pinMode(MT1_D2, OUTPUT);
  pinMode(MT1_PWM, OUTPUT);
  pinMode(MT1_HALL_A, INPUT_PULLUP);
  pinMode(MT1_HALL_B, INPUT_PULLUP);

  // motor2 
  pinMode(MT2_D1, OUTPUT);
  pinMode(MT2_D2, OUTPUT);
  pinMode(MT2_PWM, OUTPUT);
  pinMode(MT2_HALL_A, INPUT_PULLUP);
  pinMode(MT2_HALL_B, INPUT_PULLUP);

  // linear actuator
  pinMode(Ln_Act_D1, OUTPUT);
  pinMode(Ln_Act_D2, OUTPUT);
  pinMode(Ln_Act_PWM, OUTPUT);

  delay(200);
  
  // motor 1
  digitalWrite(MT1_D1, HIGH);
  delay(1000);
  digitalWrite(MT1_D2, HIGH);
  delay(1000);
  digitalWrite(MT1_PWM, HIGH);

  // motor 2
  delay(1000);
  digitalWrite(MT2_D1, HIGH);
  delay(1000);
  digitalWrite(MT2_D2, HIGH);
  delay(1000);
  digitalWrite(MT2_PWM, HIGH);
  
  // linear actuator
  delay(1000);
  digitalWrite(Ln_Act_D1, HIGH);
  delay(1000);
  digitalWrite(Ln_Act_D2, HIGH);
  delay(1000);
  digitalWrite(Ln_Act_PWM, HIGH);

  // check done! turn off all the outputs.
  delay(2000);
  digitalWrite(MT1_D1, LOW);
  digitalWrite(MT1_D2, LOW);
  digitalWrite(MT1_PWM, LOW);
  digitalWrite(MT2_D1, LOW);
  digitalWrite(MT2_D2, LOW);
  digitalWrite(MT2_PWM, LOW);
  digitalWrite(Ln_Act_D1, LOW);
  digitalWrite(Ln_Act_D2, LOW);
  digitalWrite(Ln_Act_PWM, LOW);
}

// check full setup
void debug_pins_setup_full(){
  setupMotorOne();
  setupMotorTwo();
  setupLinearActuator();
}