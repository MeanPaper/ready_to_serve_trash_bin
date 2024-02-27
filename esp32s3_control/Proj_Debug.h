#ifndef PROJ_DEBUG_H
#define PROJ_DEBUG_H

// this is library that contain all the debug functions

// check if each pin can do the designated data flow (in/out)
void debug_pins_basic();

// check if the assign pin is outputing desired pwm signal
void debug_pins_pwm();

// check hall encoder reading
void debug_pins_hall();

#endif