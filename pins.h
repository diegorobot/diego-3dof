# include "Arduino.h"

#ifndef PINS_H
#define PINS_H

/****************************************************************************************
*  Arduino UNO pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 1

#ifndef __AVR_ATmega328P__
#error Oops!  Make sure you have 'Arduino Duemilanove w/ ATMega328' selected from the 'Tools -> Boards' menu.
#endif

#define ENABLE_PIN         8

#define X_STEP_PIN         2
#define X_DIR_PIN          5
#define X_MIN_PIN          -1
#define X_MAX_PIN         -1

#define Y_STEP_PIN          3
#define Y_DIR_PIN           6
#define Y_MIN_PIN          -1
#define Y_MAX_PIN         -1

#define Z_STEP_PIN          4
#define Z_DIR_PIN           7
#define Z_MIN_PIN          -1
#define Z_MAX_PIN         -1
#define KILL_PIN            12

#endif

#endif



