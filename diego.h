#ifndef DIEGO_H
#define DIEGO_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "Configuration.h"
#include "pins.h"
# include "Arduino.h"

void get_command();
void process_commands();

void manage_inactivity();

#define enable() WRITE(ENABLE_PIN,LOW)
#define disable() WRITE(ENABLE_PIN,HIGH)

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2};

void get_coordinates();
void prepare_move();
void kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;

extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins

void WRITE(short pin, short value);
bool READ(short pin);
void SET_INPUT(short pin);
void SET_OUTPUT(short pin);



#endif
