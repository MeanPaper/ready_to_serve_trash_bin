#ifndef _PROJ_DEBUG_H_
#define _PROJ_DEBUG_H_

#include "Proj_Setup.h"

// this is library that contain all the debug functions

// check if each pin can do the designated data flow (in/out)
void debug_pins_basic();

// check if the assign pin is outputing desired pwm signal
void debug_pins_pwm();

// check hall encoder reading
void debug_pins_hall();

// check full setup
void debug_pins_setup_full();

#endif