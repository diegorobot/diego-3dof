/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*
  Reasoning behind the mathematics in this module (in the key of 'Mathematica'):

  s == speed, a == acceleration, t == time, d == distance

  Basic definitions:

  Speed[s_, a_, t_] := s + (a*t)
  Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]

  Distance to reach a specific speed with a constant acceleration:

  Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
  d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()

  Speed after a given distance of travel with constant acceleration:

  Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
  m -> Sqrt[2 a d + s^2]

  DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]

  When to start braking (di) to reach a specified destionation speed (s2) after accelerating
  from initial speed s1 without ever stopping at a plateau:

  Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
  di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()

  IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
*/

#include "diego.h"
#include "planner.h"
#include "stepper.h"
#include "Configuration.h"

//===========================================================================
//=============================public variables ============================
//===========================================================================

unsigned long minsegmenttime;
float max_feedrate[3]; // set the max speeds
float axis_steps_per_unit[3];
unsigned long max_acceleration_units_per_sq_second[3]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
float max_z_jerk;
//float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

// The current position of the tool in absolute steps
long position[3];   //rescaled from extern when axis_steps_per_unit are changed by gcode
float position_mm[3]; //
static float previous_speed[3]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment


//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//=============================private variables ============================
//===========================================================================

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) {
    block_index = 0;
  }
  return (block_index);
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) {
    block_index = BLOCK_BUFFER_SIZE;
  }
  block_index--;
  return (block_index);
}

//===========================================================================
//=============================functions         ============================
//===========================================================================

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration != 0) {
    return ((target_rate * target_rate - initial_rate * initial_rate) /
            (2.0 * acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance)
{
  if (acceleration != 0) {
    return ((2.0 * acceleration * distance - initial_rate * initial_rate + final_rate * final_rate) /
            (4.0 * acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate * entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate * exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if (initial_rate < 120) {
    initial_rate = 120;
  }
  if (final_rate < 120) {
    final_rate = 120;
  }

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration));

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count); //(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }


  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;

  }
  CRITICAL_SECTION_END;
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if (!current) {
    return;
  }

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
                                    max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
      }
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;

  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  if (((block_buffer_head - tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = {
      NULL, NULL, NULL
    };
    while (block_index != tail) {
      block_index = prev_block_index(block_index);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if (!previous) {
    return;
  }

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
                                max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = {
    NULL, NULL, NULL
  };

  while (block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0], block[1], block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by planner_recalculate() after
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed / current->nominal_speed,
                                      next->entry_speed / current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed / next->nominal_speed,
                                  MINIMUM_PLANNER_SPEED / next->nominal_speed);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if
//     a. The speed increase within one block would require faster accelleration than the one, true
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;

  position[X_AXIS]=DEFAULT_POINT_X_STEPS;
  position[Y_AXIS]=DEFAULT_POINT_Y_STEPS; 
  position[Z_AXIS]=(int32_t)(DEFAULT_POINT_Z_STEPS+0.5);

  plan_set_current_position_mm(DEFAULT_POINT_X_MM, DEFAULT_POINT_Y_MM, DEFAULT_POINT_Z_MM);

  previous_nominal_speed = 0.0;
}


void check_axes_activity()
{
  unsigned char x_active = 0;
  unsigned char y_active = 0;
  unsigned char z_active = 0;


  block_t *block;

  if (block_buffer_tail != block_buffer_head)
  {
    uint8_t block_index = block_buffer_tail;
    while (block_index != block_buffer_head)
    {
      block = &block_buffer[block_index];
      if (block->steps_x != 0) x_active++;
      if (block->steps_y != 0) y_active++;
      if (block->steps_z != 0) z_active++;
      block_index = (block_index + 1) & (BLOCK_BUFFER_SIZE - 1);
    }
  }
  //  if((x_active==0) && (y_active ==0) && (z_active==0)) disable();
}


float junction_deviation = 0.1;
// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(const float &x, const float &y, const float &z, float feed_rate)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  Serial.print("------in plan_buffer_line\r\n");
  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head)
  {
    manage_inactivity();
  }

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[3];
//  ////the pos for C
  float c_z = z + DEFAULT_DE_LENGTH_MM;
  float c_x = x;
  float c_y = y;
  ///////////////diego_3dof kinemitacs///////////////////////
  ///alpha  oa_map_xy to y
  float alpha = 0;
  if(y==0){///////////in the x line
    if(x>0){
      alpha=PI/2;
      c_x=x-DEFAULT_CD_LENGTH_MM;
    }else if(x<0){
      alpha=-PI/2;
      c_x=x+DEFAULT_CD_LENGTH_MM;
    }
  }else{
    float tan_alpha=x/y;
    if(tan_alpha==0){///in the y line
      if(y>0){
        alpha=0;
        c_y = y - DEFAULT_CD_LENGTH_MM;
      }else{
        alpha=PI;
        c_y = y + DEFAULT_CD_LENGTH_MM;
      }
    }else if(tan_alpha>0){
      if(y>0){
        alpha=atan(tan_alpha);
      }else{
        alpha=atan(tan_alpha)-PI;
      }
      c_y=y-DEFAULT_CD_LENGTH_MM*cos(alpha);
      c_x=x-DEFAULT_CD_LENGTH_MM*sin(alpha);
    }else{
      if(y>0){
        alpha=atan(tan_alpha);
      }else{
        alpha=PI/2-atan(tan_alpha);
      }
      c_y=y-DEFAULT_CD_LENGTH_MM*cos(alpha);
      c_x=x-DEFAULT_CD_LENGTH_MM*sin(alpha);
    }
  }

  Serial.print("------------alpha\r\n");
  Serial.print(alpha*180/PI);
  Serial.print("\r\n");
  //gamma 180- ab to bc
  float gamma = 0;
  float default_oa = DEFAULT_OA_LENGTH_MM;
  float default_ab = DEFAULT_AB_LENGTH_MM;
  float default_bc = DEFAULT_BC_LENGTH_MM;
  float default_theta = DEFAULT_THETA_DEGREE;

  float a_x = default_oa * cos(default_theta) * sin(alpha);
  float a_y = default_oa * cos(default_theta) * cos(alpha);
  float a_z = default_oa * sin(default_theta);

  float ac = sqrt(pow(c_x - a_x, 2) + pow(c_y - a_y, 2) + pow(c_z - a_z, 2));

  float cos_gamma = (pow(default_ab, 2) + pow(default_bc, 2) - pow(ac, 2)) / (2 * default_ab * default_bc);
//  Serial.print("-------------cos_gamma");
//  Serial.print(cos_gamma);

  
  //gamma = PI - acos(cos_gamma);
  gamma = acos(cos_gamma);

  Serial.print("-------------gamma\r\n");
  Serial.print(gamma*180/PI);
  Serial.print("\r\n");

  ///beta; map to the axis between oa to ab
  float beta = 0;
  float k_map_oa = sin(default_theta) / cos(default_theta);

  float a_map_x = default_oa * cos(default_theta);  ////62.47
  float a_map_y = default_oa * sin(default_theta);  ////101.7

  float c_map_x = sqrt(pow(c_x, 2) + pow(c_y, 2)); ////130
  float c_map_y = c_z; ////102
  float k_map_ac = (c_map_y - a_map_y) / (c_map_x - a_map_x);
//  Serial.print("-------------c_map_y\r\n");
//  Serial.print(c_map_y);
//  Serial.print("\r\n");
//  Serial.print("-------------a_map_y\r\n");
//  Serial.print(a_map_y);
//  Serial.print("\r\n");
//  Serial.print("-------------c_map_x\r\n");
//  Serial.print(c_map_x);
//  Serial.print("\r\n");
//  Serial.print("-------------a_map_x\r\n");
//  Serial.print(a_map_x);
//  Serial.print("\r\n");
//  Serial.print("-------------k_map_oa\r\n");
//  Serial.print(k_map_oa);
//  Serial.print("\r\n");
//  Serial.print("-------------k_map_ac\r\n");
//  Serial.print(k_map_ac);
//  Serial.print("\r\n");

  float tan_oa_ac = abs(k_map_oa - k_map_ac) / abs(1 - k_map_oa * k_map_ac);
  float cos_ab_ac = (pow(default_ab, 2) + pow(ac, 2) - pow(default_bc, 2)) / (2 * default_ab*ac);
  float ab_ac = acos(cos_ab_ac);
//  Serial.print("-------------ab_ac\r\n");
//  Serial.print(cos_ab_ac);
//  Serial.print("\r\n");
//  Serial.print(ab_ac);
//  Serial.print("\r\n");
  float ac_oa = atan(tan_oa_ac);
//  Serial.print("-------------ac_oa\r\n");
//  Serial.print(ac_oa);
//  Serial.print("\r\n");
  
  if (ac_oa < ab_ac) {
    if (k_map_ac > k_map_oa) {
      beta = 0 - ab_ac - atan(tan_oa_ac);
    } else {
      beta = 0 - ab_ac + atan(tan_oa_ac);
    }
  } else {
    float oc = sqrt(pow(c_map_x, 2) + pow(c_map_y, 2));
    float cos_oa_ac = (pow(default_oa, 2) + pow(ac, 2) - pow(oc, 2)) / (2 * default_oa * ac);
    float oa_ac = acos(cos_oa_ac);
    beta = PI - oa_ac - ab_ac;
  }

  Serial.print("-------------beta\r\n");
  Serial.print(beta*180/PI);
  Serial.print("\r\n");

  float degree_per_step_axis1 = 2*PI / (DEFAULT_STEPS_PER_CYCLE_AXIS_1 * DEFAULT_REDUCTION_RATE_AXIS_1 * DEFAULT_MICROSTEPS_AXIS_1); //steps per cycle
  float degree_per_step_axis2 = 2*PI / (DEFAULT_STEPS_PER_CYCLE_AXIS_2 * DEFAULT_REDUCTION_RATE_AXIS_2 * DEFAULT_MICROSTEPS_AXIS_2); //steps per cycle
  float degree_per_step_axis3 = 2*PI / (DEFAULT_STEPS_PER_CYCLE_AXIS_3 * DEFAULT_REDUCTION_RATE_AXIS_3 * DEFAULT_MICROSTEPS_AXIS_3); //steps per cycle

  float axis1_degree_steps = alpha / degree_per_step_axis1;
  float axis2_degree_steps = beta / degree_per_step_axis2;
  float axis3_degree_steps = gamma / degree_per_step_axis3;

  target[X_AXIS] = (int32_t)(axis1_degree_steps + 0.5);
  target[Y_AXIS] = (int32_t)(axis2_degree_steps + 0.5);
  target[Z_AXIS] = (int32_t)(axis3_degree_steps + 0.5); 
  
//  Serial.print("-------------target[X_AXIS]\r\n");
//  Serial.print(target[X_AXIS]);
//  Serial.print("\r\n");  
//  Serial.print("-------------target[Y_AXIS]\r\n");
//  Serial.print(target[Y_AXIS]); 
//  Serial.print("\r\n");
//  Serial.print("-------------target[Z_AXIS]\r\n");
//  Serial.print(target[Z_AXIS]);
//  Serial.print("\r\n");

  

  /////////////////////////////////////////////////////////

  //target[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  //target[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  //target[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);

  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS] - position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS] - position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS] - position[Z_AXIS]);
//  Serial.print("-------------block->steps_x\r\n");
//  Serial.print(block->steps_x);
//  Serial.print("\r\n");
//  Serial.print("-------------block->steps_y\r\n");
//  Serial.print(block->steps_y);
//  Serial.print("\r\n");
//  Serial.print("-------------block->steps_z\r\n");
//  Serial.print(block->steps_z);
//  Serial.print("\r\n");

  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments)
  {
    return;
  }

  // Compute direction bits for this block
  block->direction_bits = 0;
  if (target[X_AXIS] > position[X_AXIS])
  {
    block->direction_bits |= (1 << X_AXIS);
  }
  if (target[Y_AXIS] > position[Y_AXIS])
  {
    block->direction_bits |= (1 << Y_AXIS);
  }
  if (target[Z_AXIS] < position[Z_AXIS])
  {
    block->direction_bits |= (1 << Z_AXIS);
  }

  //enable active axes
  if ((block->steps_x != 0) || (block->steps_y != 0) || (block->steps_z != 0)) enable();
  //  if (block->steps_x != 0) enable_x();
  //  if (block->steps_y != 0) enable_y();
  //  if (block->steps_z != 0) enable_z();

  if (feed_rate < mintravelfeedrate) feed_rate = mintravelfeedrate;

  float delta_mm[3];
  //delta_mm[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / axis_steps_per_unit[X_AXIS];
  //delta_mm[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / axis_steps_per_unit[Y_AXIS];
  //delta_mm[Z_AXIS] = (target[Z_AXIS] - position[Z_AXIS]) / axis_steps_per_unit[Z_AXIS];

  ////diego
  delta_mm[X_AXIS] = x - position_mm[X_AXIS];
  delta_mm[Y_AXIS] = y - position_mm[Y_AXIS];
  delta_mm[Z_AXIS] = z - position_mm[Z_AXIS];

  block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  float inverse_millimeters = 1.0 / block->millimeters; // Inverse millimeters to remove multiple divides

  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[3];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for (int i = 0; i < 3; i++)
  {
    current_speed[i] = delta_mm[i] * inverse_second;
    if (fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  // Correct the speed
  if ( speed_factor < 1.0)
  {
    for (unsigned char i = 0; i < 3; i++)
    {
      current_speed[i] *= speed_factor;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;
  if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else
  {
    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    if (((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    //    if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
    //      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if (((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * 8.388608);

#if 0  // Use old jerk for now
  // Compute path unit vector
  double unit_vec[3];

  unit_vec[X_AXIS] = delta_mm[X_AXIS] * inverse_millimeters;
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS] * inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS] * inverse_millimeters;

  // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
  // Let a circle be tangent to both previous and current path line segments, where the junction
  // deviation is defined as the distance from the junction to the closest edge of the circle,
  // colinear with the circle center. The circular segment joining the two paths represents the
  // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
  // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
  // path width or max_jerk in the previous grbl version. This approach does not actually deviate
  // from path, but used as a robust way to compute cornering speeds, as it takes into account the
  // nonlinearities of both the junction angle and junction velocity.
  double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                       - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                       - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;

    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95) {
      vmax_junction = min(previous_nominal_speed, block->nominal_speed);
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        double sin_theta_d2 = sqrt(0.5 * (1.0 - cos_theta)); // Trig half angle identity. Always positive.
        vmax_junction = min(vmax_junction,
                            sqrt(block->acceleration * junction_deviation * sin_theta_d2 / (1.0 - sin_theta_d2)) );
      }
    }
  }
#endif
  // Start with a safe speed
  float vmax_junction = max_xy_jerk / 2;
  float vmax_junction_factor = 1.0;
  if (fabs(current_speed[Z_AXIS]) > max_z_jerk / 2)
    vmax_junction = min(vmax_junction, max_z_jerk / 2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS] - previous_speed[X_AXIS]), 2) + pow((current_speed[Y_AXIS] - previous_speed[Y_AXIS]), 2));
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk / jerk);
    }
    if (fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_z_jerk / fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    }

    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) {
    block->nominal_length_flag = true;
  }
  else {
    block->nominal_length_flag = false;
  }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed,
                                safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]
  plan_set_current_position_mm(x, y, z);

  planner_recalculate();
  st_wake_up();
}

void plan_set_position(const float &x, const float &y, const float &z)
{
  position[X_AXIS] = lround(x * axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = lround(y * axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = lround(z * axis_steps_per_unit[Z_AXIS]);
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS]);
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
}
uint8_t movesplanned()
{
  return (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}
void plan_set_current_position_mm(float x, float y, float z)
{
  position_mm[X_AXIS] = x;
  position_mm[Y_AXIS] = y;
  position_mm[Z_AXIS] = z;
}


