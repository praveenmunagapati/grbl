/*
  spindle_control.c - pen servo control methods
  Part of Grbl - Pen Plotter Edition
 
  PEN_SERVO update by Bart Dring 8/2017
  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
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
 
#include "grbl.h"
 
/*
Pen Servo: 
 
For a pen bot, the spindle PWM controls a servo.
In stepper.c it looks at the Z position. Any Z>0 is pen up.
 
Use 1024 prescaler: 16,000,000 MHz / 1024 = 15625 Hz
8 bit timer: 15625 / 256 = 61 Hz (~50Hz for servos)
Each tick = 0.000064sec 
Servo 0° = 0.001 sec (0.001 / 0.000064 = ~16 ticks)
Servo 180° = 0.002 sec (0.002 / 0.000064 = ~31 ticks)
*/

// Servo position constants - adjust for your servo travel
// If servo moves the wrong way, swap these values
#define PEN_SERVO_DOWN     31      
#define PEN_SERVO_UP       16        

// Dynamic pen pressure: Z-depth range that maps to full servo travel.
// Z >= 0 = pen up, Z = -PEN_PRESSURE_MAX_DEPTH = full pressure (servo fully down).
// Values between 0 and -max are linearly interpolated for variable brush strokes.
#define PEN_PRESSURE_MAX_DEPTH  5.0  // mm (Z = -5.0 = maximum pressure)
 
// Servo hardware defines (Timer2 OC2A on Digital Pin 11)
#define SERVO_PWM_DDR     DDRB
#define SERVO_PWM_PORT    PORTB
#define SERVO_PWM_BIT     3    // Uno Digital Pin 11 
#define SERVO_TCCRA_REGISTER    TCCR2A
#define SERVO_TCCRB_REGISTER    TCCR2B
#define SERVO_OCR_REGISTER      OCR2A
#define SERVO_COMB_BIT          COM2A1
 
void init_servo()
{
  SERVO_PWM_DDR |= (1<<SERVO_PWM_BIT); // Configure as output pin.
  SERVO_TCCRA_REGISTER = (1<<COM2A1) | ((1<<WGM20) | (1<<WGM21));
  SERVO_TCCRB_REGISTER = (1<<CS22) | (1 <<CS21) | (1<<CS20);
  set_pen_pos();
}	

void pen_up()
{
  SERVO_OCR_REGISTER = PEN_SERVO_UP;
}

void pen_down()
{
  SERVO_OCR_REGISTER = PEN_SERVO_DOWN;
}

// Set servo to a specific tick value (clamped to valid range)
void pen_set_pressure(uint8_t tick)
{
  if (tick < PEN_SERVO_UP) { tick = PEN_SERVO_UP; }
  if (tick > PEN_SERVO_DOWN) { tick = PEN_SERVO_DOWN; }
  SERVO_OCR_REGISTER = tick;
}

void set_pen_pos()
{
  float wpos_z;
  wpos_z = system_convert_axis_steps_to_mpos(sys_position, Z_AXIS) - gc_state.coord_system[Z_AXIS];
  
  if (wpos_z >= 0.0) {
    // Pen up: Z is at or above zero
    pen_up();
  } else {
    // Dynamic pressure: map Z depth to servo angle
    // Z = 0 -> PEN_SERVO_UP, Z = -MAX_DEPTH -> PEN_SERVO_DOWN
    float depth = -wpos_z; // Make positive
    if (depth > PEN_PRESSURE_MAX_DEPTH) { depth = PEN_PRESSURE_MAX_DEPTH; }
    
    // Linear interpolation: UP + (DOWN - UP) * (depth / max_depth)
    float ratio = depth / PEN_PRESSURE_MAX_DEPTH;
    uint8_t tick = PEN_SERVO_UP + (uint8_t)((float)(PEN_SERVO_DOWN - PEN_SERVO_UP) * ratio);
    pen_set_pressure(tick);
  }	
}

 
void spindle_init()
{
  // Configure spindle enable pin as output.
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  spindle_stop();
  init_servo(); // Initialize pen servo
}
 
 
uint8_t spindle_get_state()
{
  if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {        
    return(SPINDLE_STATE_CW);
  }
  return(SPINDLE_STATE_DISABLE);
}
 
 
void spindle_stop()
{
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
}

 
void _spindle_set_state(uint8_t state)
{
  if (sys.abort) { return; } // Block during abort.
  if (state == SPINDLE_DISABLE) {
    spindle_stop();
  } else {
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
  }
  sys.report_ovr_counter = 0; // Set to report change immediately
}

void _spindle_sync(uint8_t state)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  _spindle_set_state(state);
}
