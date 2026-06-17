/*
  gcode.c - rs274/ngc parser.
  Part of Grbl - Pen Plotter Edition

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // Limited by max unsigned 8-bit value

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2

// Declare gc extern struct
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);


void gc_init()
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // Load default G54 coordinate system.
  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) {
    report_status_message(STATUS_SETTING_READ_FAIL);
  }
}


// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
void gc_sync_position()
{
  system_convert_array_steps_to_mpos(gc_state.position,sys_position);
}


// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. In this function, all units and positions are converted and
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine
// coordinates, respectively.
uint8_t gc_execute_line(char *line)
{
  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parser and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new
     block. This struct contains all of the necessary information to execute the block. */

  memset(&gc_block, 0, sizeof(parser_block_t)); // Initialize the parser block struct.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); // Copy current modes

  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; // Tracks G10 P coordinate selection for execution

  // Initialize bitflag tracking variables for axis indices compatible operations.
  uint8_t axis_words = 0; // XYZ tracking
  uint8_t ijk_words = 0; // IJK tracking

  // Initialize command and value words and parser flags variables.
  uint16_t command_words = 0; // Tracks G and M command words. Also used for modal group violations.
  uint16_t value_words = 0; // Tracks value words.
  uint8_t gc_parser_flags = GC_PARSER_NONE;

  // Determine if the line is a jogging motion or a normal g-code block.
  if (line[0] == '$') { // NOTE: `$J=` already parsed when passed to this function.
    // Set G1 and G94 enforced modes to ensure accurate error checks.
    gc_parser_flags |= GC_PARSER_JOG_MOTION;
    gc_block.modal.motion = MOTION_MODE_LINEAR;
    gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
    #ifdef USE_LINE_NUMBERS
      gc_block.values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
    #endif
  }

  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block line. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also,
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */

  uint8_t word_bit; // Bit-value for assigning tracking variables
  uint8_t char_counter;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) { char_counter = 3; } // Start parsing after `$J=`
  else { char_counter = 0; }

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.

    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value]

    // Convert values to smaller uint8 significand and mantissa values for parsing this word.
    int_value = trunc(value);
    mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.
    // NOTE: Rounding must be used to catch small floating point errors.

    // Check if the g-code word is supported or errors due to modal group violations or has
    // been repeated in the g-code block. If ok, update the command or record its value.
    switch(letter) {

      /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
         NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */

      case 'G':
        // Determine 'G' command and its modal group
        switch(int_value) {
          case 10: case 28: case 30: case 92:
            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            // No break. Continues to next line.
          case 4: case 53:
            word_bit = MODAL_GROUP_G0;
            gc_block.non_modal_command = int_value;
            if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
              if (!((mantissa == 0) || (mantissa == 10))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); }
              gc_block.non_modal_command += mantissa;
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
            }                
            break;
          case 0: case 1: case 2: case 3: case 5:
            // Check for G0/1/2/3/5 being called with G10/28/30/92 on same block.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE;
            // No break. Continues to next line.
          case 80:
            word_bit = MODAL_GROUP_G1;
            gc_block.modal.motion = int_value;
            break;
          case 17: case 18: case 19:
            word_bit = MODAL_GROUP_G2;
            gc_block.modal.plane_select = int_value - 17;
            break;
          case 90: case 91:
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3;
              gc_block.modal.distance = int_value - 90;
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
              // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
            }
            break;
          case 93: case 94:
            word_bit = MODAL_GROUP_G5;
            gc_block.modal.feed_rate = 94 - int_value;
            break;
          case 20: case 21:
            word_bit = MODAL_GROUP_G6;
            gc_block.modal.units = 21 - int_value;
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            word_bit = MODAL_GROUP_G12;
            gc_block.modal.coord_select = int_value - 54; // Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G61.1 not supported]
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G command]
        }
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [Unsupported or invalid Gxx.x command]
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      case 'M':
        // Determine 'M' command and its modal group
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
              case 1: break; // Optional stop not supported. Ignore.
              default: gc_block.modal.program_flow = int_value; // Program end and reset
            }
            break;
          case 3: case 4: case 5:
            word_bit = MODAL_GROUP_M7;
            switch(int_value) {
              case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;
              case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;
              case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;
            }
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
        }

        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      // NOTE: All remaining letters assign values.
      default:
        switch(letter){
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          case 'I': word_bit = WORD_I; gc_block.values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          case 'Q': word_bit = WORD_Q; gc_block.values.q = value; break;
          case 'R': word_bit = WORD_R; gc_block.values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block.values.s = value; break;
          case 'T': word_bit = WORD_T; 
					  if (value > MAX_TOOL_NUMBER) { FAIL(STATUS_GCODE_MAX_VALUE_EXCEEDED); }
            gc_block.values.t = int_value;
						break;
          case 'X': word_bit = WORD_X; gc_block.values.xyz[X_AXIS] = value; axis_words |= (1<<X_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block.values.xyz[Y_AXIS] = value; axis_words |= (1<<Y_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block.values.xyz[Z_AXIS] = value; axis_words |= (1<<Z_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }

        // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } // [Word repeated]
        // Check for invalid negative values for words F, N, P, T, and S.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } // [Word value cannot be negative]
        }
        value_words |= bit(word_bit); // Flag to indicate parameter assigned.

    }
  }


  /* -------------------------------------------------------------------------------------
     STEP 3: Error-check all commands and values passed in this block. */

  // Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
  // command has been sent. If so, set axis command to current motion mode.
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } // Assign implicit motion-mode
  }

  // Check for valid line number N value.
  if (bit_istrue(value_words,bit(WORD_N))) {
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [Exceeds max line number]
  }

  // [2. Set feed rate mode ]:
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); }
    if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
  } else {
    if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
      if (axis_command == AXIS_COMMAND_MOTION_MODE) {
        if ((gc_block.modal.motion != MOTION_MODE_NONE) && (gc_block.modal.motion != MOTION_MODE_SEEK)) {
          if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [F word missing]
        }
      }
    } else { // = G94
      if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { // Last state is also G94
        if (bit_istrue(value_words,bit(WORD_F))) {
          if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
        } else {
          gc_block.values.f = gc_state.feed_rate; // Push last state feed rate
        }
      }
    }
  }

  // [4. Set spindle speed ]: S is negative (done.)
  if (bit_isfalse(value_words,bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }

  // [10. Dwell ]: P value missing. P is negative (done.)
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words,bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P word missing]
    bit_false(value_words,bit(WORD_P));
  }

  // [11. Set active plane ]: N/A
  switch (gc_block.modal.plane_select) {
    case PLANE_SELECT_XY:
      axis_0 = X_AXIS;
      axis_1 = Y_AXIS;
      axis_linear = Z_AXIS;
      break;
    case PLANE_SELECT_ZX:
      axis_0 = Z_AXIS;
      axis_1 = X_AXIS;
      axis_linear = Y_AXIS;
      break;
    default: // case PLANE_SELECT_YZ:
      axis_0 = Y_AXIS;
      axis_1 = Z_AXIS;
      axis_linear = X_AXIS;
  }

  // [12. Set length units ]: N/A
  // Pre-convert XYZ coordinate values to millimeters, if applicable.
  uint8_t idx;
  if (gc_block.modal.units == UNITS_MODE_INCHES) {
    for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
      if (bit_istrue(axis_words,bit(idx)) ) {
        gc_block.values.xyz[idx] *= MM_PER_INCH;
      }
    }
  }

  // [15. Coordinate system selection ]
  float block_coord_system[N_AXIS];
  memcpy(block_coord_system,gc_state.coord_system,sizeof(gc_state.coord_system));
  if ( bit_istrue(command_words,bit(MODAL_GROUP_G12)) ) { // Check if called in block
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys]
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
      if (!(settings_read_coord_data(gc_block.modal.coord_select,block_coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
    }
  }

  // [19. Remaining non-modal actions ]
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS) }; // [No axis words]
      if (bit_isfalse(value_words,((1<<WORD_P)|(1<<WORD_L)))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P/L word missing]
      coord_select = trunc(gc_block.values.p); // Convert p value to int.
      if (coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys]
      if (gc_block.values.l != 20) {
        if (gc_block.values.l == 2) {
          if (bit_istrue(value_words,bit(WORD_R))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G10 L2 R not supported]
        } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported L]
      }
      bit_false(value_words,(bit(WORD_L)|bit(WORD_P)));

      // Determine coordinate system to change and try to load from EEPROM.
      if (coord_select > 0) { coord_select--; } // Adjust P1-P6 index to EEPROM coordinate data indexing.
      else { coord_select = gc_block.modal.coord_select; } // Index P0 as the active coordinate system
      
      // NOTE: Store parameter data in IJK values. By rule, they are not in use with this command.
      if (!settings_read_coord_data(coord_select,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); } // [EEPROM read fail]

      // Pre-calculate the coordinate data changes.
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
        if (bit_istrue(axis_words,bit(idx)) ) {
          if (gc_block.values.l == 20) {
            // L20: Update coordinate system axis at current position (with modifiers) with programmed value
            // WPos = MPos - WCS - G92 -> WCS = MPos - G92 - WPos
            gc_block.values.ijk[idx] = gc_state.position[idx]-gc_state.coord_offset[idx]-gc_block.values.xyz[idx];
          } else {
            // L2: Update coordinate system axis to programmed value.
            gc_block.values.ijk[idx] = gc_block.values.xyz[idx];
          }
        }
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]

      // Update axes defined only in block. Offsets current system to defined value.
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
        if (bit_istrue(axis_words,bit(idx)) ) {
          // WPos = MPos - WCS - G92 -> G92 = MPos - WCS - WPos
          gc_block.values.xyz[idx] = gc_state.position[idx]-block_coord_system[idx]-gc_block.values.xyz[idx];
        } else {
          gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
        }
      }
      break;

    default:
      if (axis_words) {
        for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space.
          if ( bit_isfalse(axis_words,bit(idx)) ) {
            gc_block.values.xyz[idx] = gc_state.position[idx]; // No axis word in block. Keep same axis position.
          } else {
            // Update specified value according to distance mode or ignore if absolute override is active.
            if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
              // Apply coordinate offsets based on distance mode.
              if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {
                gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
              } else {  // Incremental mode
                gc_block.values.xyz[idx] += gc_state.position[idx];
              }
            }
          }
        }
      }

      // Check remaining non-modal commands for errors.
      switch (gc_block.non_modal_command) {
        case NON_MODAL_GO_HOME_0: // G28
        case NON_MODAL_GO_HOME_1: // G30
          // Retreive G28/30 go-home position data (in machine coordinates) from EEPROM
          if (gc_block.non_modal_command == NON_MODAL_GO_HOME_0) {
            if (!settings_read_coord_data(SETTING_INDEX_G28,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          } else { // == NON_MODAL_GO_HOME_1
            if (!settings_read_coord_data(SETTING_INDEX_G30,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          }
          if (axis_words) {
            // Move only the axes specified in secondary move.
            for (idx=0; idx<N_AXIS; idx++) {
              if (!(axis_words & (1<<idx))) { gc_block.values.ijk[idx] = gc_state.position[idx]; }
            }
          } else {
            axis_command = AXIS_COMMAND_NONE; // Set to none if no intermediate motion.
          }
          break;
        case NON_MODAL_SET_HOME_0: // G28.1
        case NON_MODAL_SET_HOME_1: // G30.1
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET:
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 not active]
          }
          break;
      }
  }

  // [20. Motion modes ]:
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
    if (axis_words) { FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); } // [No axis words allowed]

  } else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {

    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

    } else {
      if (gc_block.values.f == 0.0) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [Feed rate undefined]

      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR:
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }
          break;
        case MOTION_MODE_CUBIC_SPLINE:
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); }
          if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); }
          
          // G5 requires I, J, P, Q words for control points
          if (!(value_words & (bit(WORD_I)|bit(WORD_J)|bit(WORD_P)|bit(WORD_Q)))) { FAIL(STATUS_GCODE_INVALID_TARGET); }
          bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_P)|bit(WORD_Q)));
          
          // Convert control point values to proper units.
          if (gc_block.modal.units == UNITS_MODE_INCHES) {
             gc_block.values.ijk[axis_0] *= MM_PER_INCH;
             gc_block.values.ijk[axis_1] *= MM_PER_INCH;
             gc_block.values.p *= MM_PER_INCH;
             gc_block.values.q *= MM_PER_INCH;
          }
          break;
        case MOTION_MODE_CW_ARC: 
          gc_parser_flags |= GC_PARSER_ARC_IS_CLOCKWISE; // No break intentional.
        case MOTION_MODE_CCW_ARC:
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); } // [No axis words in plane]

          // Calculate the change in position along each selected axis
          float x,y;
          x = gc_block.values.xyz[axis_0]-gc_state.position[axis_0]; // Delta x between current position and target
          y = gc_block.values.xyz[axis_1]-gc_state.position[axis_1]; // Delta y between current position and target

          if (value_words & bit(WORD_R)) { // Arc Radius Mode
            bit_false(value_words,bit(WORD_R));
            if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]

            // Convert radius value to proper units.
            if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }

            float h_x2_div_d = 4.0 * gc_block.values.r*gc_block.values.r - x*x - y*y;

            if (h_x2_div_d < 0) { FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); } // [Arc radius error]

            h_x2_div_d = -sqrt(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
            if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

            if (gc_block.values.r < 0) {
                h_x2_div_d = -h_x2_div_d;
                gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc
            }
            // Complete the operation by calculating the actual center of the arc
            gc_block.values.ijk[axis_0] = 0.5*(x-(y*h_x2_div_d));
            gc_block.values.ijk[axis_1] = 0.5*(y+(x*h_x2_div_d));

          } else { // Arc Center Format Offset Mode
            if (!(ijk_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE); } // [No offsets in plane]
            bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_K)));

            // Convert IJK values to proper units.
            if (gc_block.modal.units == UNITS_MODE_INCHES) {
              for (idx=0; idx<N_AXIS; idx++) {
                if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
              }
            }

            // Arc radius from center to target
            x -= gc_block.values.ijk[axis_0]; // Delta x between circle center and target
            y -= gc_block.values.ijk[axis_1]; // Delta y between circle center and target
            float target_r = hypot_f(x,y);

            // Compute arc radius for mc_arc. Defined from current location to center.
            gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);

            // Compute difference between current location and target radii for final error-checks.
            float delta_r = fabs(target_r-gc_block.values.r);
            if (delta_r > 0.005) {
              if (delta_r > 0.5) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.5mm
              if (delta_r > (0.001*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.005mm AND 0.1% radius
            }
          }
          break;
      }
    }
  }

  // [21. Program flow ]: No error checks required.

  // [0. Non-specific error-checks]: Complete unused value words check.
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    bit_false(value_words,(bit(WORD_N)|bit(WORD_F)));
  } else {
    bit_false(value_words,(bit(WORD_N)|bit(WORD_F)|bit(WORD_S)|bit(WORD_T))); // Remove single-meaning value words.
  }
  if (axis_command) { bit_false(value_words,(bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z))); } // Remove axis words.
  if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [Unused words]

  /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
  */

  // Initialize planner data struct for motion blocks.
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t)); // Zero pl_data struct

  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    if (command_words & ~(bit(MODAL_GROUP_G3) | bit(MODAL_GROUP_G6 | bit(MODAL_GROUP_G0))) ) { FAIL(STATUS_INVALID_JOG_COMMAND) };
    if (!(gc_block.non_modal_command == NON_MODAL_ABSOLUTE_OVERRIDE || gc_block.non_modal_command == NON_MODAL_NO_ACTION)) { FAIL(STATUS_INVALID_JOG_COMMAND); }

    // Initialize planner data to current spindle modal state.
    pl_data->spindle_speed = gc_state.spindle_speed;
    plan_data.condition = gc_state.modal.spindle;

    uint8_t status = jog_execute(&plan_data, &gc_block);
    if (status == STATUS_OK) { memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); }
    return(status);
  }
  
  gc_state.line_number = gc_block.values.n;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_state.line_number; // Record data for planner use.
  #endif

  // [2. Set feed rate mode ]:
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  if (gc_state.modal.feed_rate) { pl_data->condition |= PL_COND_FLAG_INVERSE_TIME; } // Set condition flag for planner use.

  // [3. Set feed rate ]:
  gc_state.feed_rate = gc_block.values.f; // Always copy this value.
  pl_data->feed_rate = gc_state.feed_rate; // Record data for planner use.

  // [4. Set spindle speed ]:
  if (gc_state.spindle_speed != gc_block.values.s) {
    if (gc_state.modal.spindle != SPINDLE_DISABLE) { 
      spindle_sync(gc_state.modal.spindle, 0.0);
    }
    gc_state.spindle_speed = gc_block.values.s; // Update spindle speed state.
  }
  pl_data->spindle_speed = gc_state.spindle_speed; // Record data for planner use.
  
  // [5. Select tool ]: NOT SUPPORTED. Only tracks tool value.
  gc_state.tool = gc_block.values.t;

  // [7. Spindle control ]:
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
    spindle_sync(gc_block.modal.spindle, pl_data->spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;
  }
  pl_data->condition |= gc_state.modal.spindle; // Set condition flag for planner use.

  // [10. Dwell ]:
  if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }

  // [11. Set active plane ]:
  gc_state.modal.plane_select = gc_block.modal.plane_select;

  // [12. Set length units ]:
  gc_state.modal.units = gc_block.modal.units;

  // [15. Coordinate system selection ]:
  if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,block_coord_system,N_AXIS*sizeof(float));
    system_flag_wco_change();
  }

  // [17. Set distance mode ]:
  gc_state.modal.distance = gc_block.modal.distance;

  // [19. Go to predefined position, Set G10, or Set axis offsets ]:
  switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
      settings_write_coord_data(coord_select,gc_block.values.ijk);
      if (gc_state.modal.coord_select == coord_select) {
        memcpy(gc_state.coord_system,gc_block.values.ijk,N_AXIS*sizeof(float));
        system_flag_wco_change();
      }
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1:
      pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // Set rapid motion condition flag.
      if (axis_command) { mc_line(gc_block.values.xyz, pl_data); }
      mc_line(gc_block.values.ijk, pl_data);
      memcpy(gc_state.position, gc_block.values.ijk, N_AXIS*sizeof(float));
      break;
    case NON_MODAL_SET_HOME_0:
      settings_write_coord_data(SETTING_INDEX_G28,gc_state.position);
      break;
    case NON_MODAL_SET_HOME_1:
      settings_write_coord_data(SETTING_INDEX_G30,gc_state.position);
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      memcpy(gc_state.coord_offset,gc_block.values.xyz,sizeof(gc_block.values.xyz));
      system_flag_wco_change();
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET:
      clear_vector(gc_state.coord_offset); // Disable G92 offsets by zeroing offset vector.
      system_flag_wco_change();
      break;
  }


  // [20. Motion modes ]:
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
      if (gc_state.modal.motion == MOTION_MODE_LINEAR) {
        mc_line(gc_block.values.xyz, pl_data);
      } else if (gc_state.modal.motion == MOTION_MODE_SEEK) {
        pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // Set rapid motion condition flag.
        mc_line(gc_block.values.xyz, pl_data);
      } else if ((gc_state.modal.motion == MOTION_MODE_CW_ARC) || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
        mc_arc(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
            axis_0, axis_1, axis_linear, bit_istrue(gc_parser_flags,GC_PARSER_ARC_IS_CLOCKWISE));
      } else if (gc_state.modal.motion == MOTION_MODE_CUBIC_SPLINE) {
        float pq[2] = {gc_block.values.p, gc_block.values.q};
        mc_bezier(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, pq, axis_0, axis_1);
      }
     
      memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
    }     
  }

  // [21. Program flow ]:
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) {
    protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
    if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
      if (sys.state != STATE_CHECK_MODE) {
        system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
        protocol_execute_realtime(); // Execute suspend.
      }
    } else { // == PROGRAM_FLOW_COMPLETED
      gc_state.modal.motion = MOTION_MODE_LINEAR;
      gc_state.modal.plane_select = PLANE_SELECT_XY;
      gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
      gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
      gc_state.modal.coord_select = 0; // G54
      gc_state.modal.spindle = SPINDLE_DISABLE;

      #ifdef RESTORE_OVERRIDES_AFTER_PROGRAM_END
        sys.f_override = DEFAULT_FEED_OVERRIDE;
        sys.r_override = DEFAULT_RAPID_OVERRIDE;
      #endif

      // Execute coordinate change and spindle stop.
      if (sys.state != STATE_CHECK_MODE) {
        if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
        system_flag_wco_change(); // Set to refresh immediately just in case something altered.
        spindle_set_state(SPINDLE_DISABLE,0.0);
      }
      report_feedback_message(MESSAGE_PROGRAM_END);
    }
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // Reset program flow.
  }

  return(STATUS_OK);
}
