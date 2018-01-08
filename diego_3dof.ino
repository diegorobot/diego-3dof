
#include "diego.h"
#include "planner.h"
#include "stepper.h"
#include "motion_control.h"
#include "pins_arduino.h"

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z 
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//===========================================================================
//=============================public variables=============================
//===========================================================================

//float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0 };
float add_homeing[3] = {0, 0, 0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z'};
static float destination[NUM_AXIS] = {0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
//const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42
//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME * 1000l;

//unsigned long starttime = 0;
//unsigned long stoptime = 0;

bool Stopped = false;


//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if (buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]), cmd);
    bufindw = (bufindw + 1) % BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if (buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy_P(&(cmdbuffer[bufindw][0]), cmd);
    bufindw = (bufindw + 1) % BUFSIZE;
    buflen += 1;
  }
}

void setup_killpin()
{
#if( KILL_PIN>-1 )
  pinMode(KILL_PIN, INPUT);
  WRITE(KILL_PIN, HIGH);
#endif
}

void setup()
{
  
  setup_killpin();
  Serial.begin(BAUDRATE);
//  Serial.print("------default gamma");
//  Serial.print(asin(0.38333333333));
//  Serial.print("------default gamma");
//  

  for (int8_t i = 0; i < NUM_AXIS; i++)
  {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }
  plan_init();  // Initialize planner;
  st_init();    // Initialize stepper, this enables interrupts!
}


void loop()
{
  if (buflen < (BUFSIZE - 1))
    get_command();

  if (buflen)
  {
    process_commands();
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
  }

  manage_inactivity();
  checkHitEndstops();
}

void get_command()
{
  while (Serial.available() > 0  && buflen < BUFSIZE) {
    serial_char = Serial.read();//MYSERIAL.read();
    if (serial_char == '\n' ||
        serial_char == '\r' ||
        (serial_char == ':' && comment_mode == false) ||
        serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if (!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if (!comment_mode) {
        comment_mode = false; //for new command
        if (strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if (gcode_N != gcode_LastN + 1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            serial_count = 0;
            return;
          }

          if (strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while (cmdbuffer[bufindw][count] != '*') checksum = checksum ^ cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if ( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else
          {
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if ((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            serial_count = 0;
            return;
          }
        }
        if ((strchr(cmdbuffer[bufindw], 'G') != NULL)) {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch ((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))) {
            case 0:
            case 1:
            case 2:
            case 3:
              if (Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
                //SERIAL_PROTOCOLLNPGM(MSG_OK);
              }
              else {
                //SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              }
              break;
            default:
              break;
          }

        }
        bufindw = (bufindw + 1) % BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if (serial_char == ';') comment_mode = true;
      if (!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

}

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  if (code_seen('G'))
  {
    switch ((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        if (Stopped == false) {
          get_coordinates(); // For X Y Z
          prepare_move();
          return;
        }
      //break;
      case 2: // G2  - CW ARC
        if (Stopped == false) {
          get_arc_coordinates();
          prepare_arc_move(true);
          return;
        }
      case 3: // G3  - CCW ARC
        if (Stopped == false) {
          get_arc_coordinates();
          prepare_arc_move(false);
          return;
        }
      case 4: // G4 dwell
        codenum = 0;
        if (code_seen('P')) codenum = code_value(); // milliseconds to wait
        if (code_seen('S')) codenum = code_value() * 1000; // seconds to wait

        st_synchronize();
        codenum += millis();  // keep track of when we started waiting
        previous_millis_cmd = millis();
        while (millis()  < codenum ) {
          manage_inactivity();
        }
        break;
      case 28: //G28 Home all Axis one at a time
        saved_feedrate = feedrate;
        saved_feedmultiply = feedmultiply;
        feedmultiply = 100;
        previous_millis_cmd = millis();
        enable_endstops(true);

        for (int8_t i = 0; i < NUM_AXIS; i++) {
          destination[i] = current_position[i];
        }
        feedrate = 0.0;
        home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        st_synchronize();
        for (int8_t i = 0; i < NUM_AXIS; i++) {
          if (code_seen(axis_codes[i])) {
            current_position[i] = code_value() + add_homeing[i];
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
          }
        }
        break;
    }
  }
}
void get_coordinates()
{
  //bool seen[4] = {false, false, false, false};
  for (int8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i]))
    {
      //destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode) * current_position[i];
      destination[i] = (float)code_value();
      //seen[i] = true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if (code_seen('F')) {
    next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
  if (code_seen('I')) {
    offset[0] = code_value();
  }
  else {
    offset[0] = 0.0;
  }
  if (code_seen('J')) {
    offset[1] = code_value();
  }
  else {
    offset[1] = 0.0;
  }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();
  // Do not use feedmultiply for E or Z only moves
  if ( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], feedrate / 60);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], feedrate * feedmultiply / 60 / 100.0);
  }
  for (int8_t i = 0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate * feedmultiply / 60 / 100.0, r, isclockwise);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for (int8_t i = 0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

void manage_inactivity()
{
  if ( (millis() - previous_millis_cmd) >  max_inactive_time )
    if (max_inactive_time)
      kill();
  if (stepper_inactive_time)  {
    if ( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if (blocks_queued() == false) {
          //disable();
      }
    }
  }
#if( KILL_PIN>-1 )
  if ( 0 == READ(KILL_PIN) )
    kill();
#endif
  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  //disable();

  while (1) {
    /* Intentionally left empty */
  } // Wait for reset
}

void Stop()
{
  if (Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
  }
}

bool IsStopped() {
  return Stopped;
};

void WRITE(short pin, short value) {
  digitalWrite(pin, value);
}

bool READ(short pin) {
  return digitalRead(pin);
}

void SET_INPUT(short pin) {
  pinMode(pin, INPUT);
}

void SET_OUTPUT(short pin) {
  pinMode(pin, OUTPUT);
}

