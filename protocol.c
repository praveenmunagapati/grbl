/*
  protocol.c - controls Grbl execution protocol and procedures
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

// Define line flags. Includes comment type tracking and line overflow detection.
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.

static void protocol_exec_rt_suspend();


/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // Perform some machine checks to make sure everything is good to go.
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limits_get_state()) {
        sys.state = STATE_ALARM; // Ensure alarm state is active.
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
  // Check for and report alarm state after a reset, error, or an initial power up.
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // Ensure alarm state is set.
  } else {
    sys.state = STATE_IDLE;
    // All systems go!
    system_execute_startup(line); // Execute startup script.
  }

  // ---------------------------------------------------------------------------------
  // Primary loop! Upon a system abort, this exits back to main() to reset the system.
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available.
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached

        protocol_execute_realtime(); // Runtime command check point.
        if (sys.abort) { return; } // Bail to calling function upon system abort

        line[char_counter] = 0; // Set string termination character.
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // Direct and execute one line of formatted input, and report status of execution.
        if (line_flags & LINE_FLAG_OVERFLOW) {
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          report_status_message(gc_execute_line(line));
        }

        // Reset tracking data for next line.
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          if (c == ')') {
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // Throw away whitespace and control characters
          } else if (c == '/') {
            // Block delete NOT SUPPORTED. Ignore character.
          } else if (c == '(') {
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    // Auto-cycle start any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.

  }

  return; /* Never reached */
}


// Block until all buffered steps are executed or in a cycle state.
void protocol_buffer_synchronize()
{
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// Auto-cycle start triggers when there is a motion ready to execute.
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // Check if there are any blocks in the buffer.
    system_set_exec_state_flag(EXEC_CYCLE_START); // If so, execute them!
  }
}


// General interface to Grbl's real-time command execution system.
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}


// Executes run-time commands, when required. This is Grbl's state machine.
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times.
  rt_exec = sys_rt_exec_alarm; // Copy volatile sys_rt_exec_alarm.
  if (rt_exec) { // Enter only if any bit flag is true
    sys.state = STATE_ALARM; // Set system alarm state
    report_alarm_message(rt_exec);
    // Halt everything upon a critical event flag.
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // Disable any existing reset
      do {
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // Clear alarm
  }

  rt_exec = sys_rt_exec_state; // Copy volatile sys_rt_exec_state.
  if (rt_exec) {

    // Execute system abort.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;
      return;
    }

    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
    // main program processes until either reset or resumed.
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SLEEP)) {

      // State check for allowable states for hold methods.
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
      
        // If in CYCLE or JOG states, immediately initiate a motion HOLD.
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) {
            st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; // Initiate suspend state with active flag.
            if (sys.state == STATE_JOG) { // Jog cancelled upon any hold event, except for sleeping.
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        // Execute and flag a motion cancel with deceleration and return to idle.
        if (rt_exec & EXEC_MOTION_CANCEL) {
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; }
        }

        // Execute a feed hold with deceleration, if required. Then, suspend system.
        if (rt_exec & EXEC_FEED_HOLD) {
          if (!(sys.state & (STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }
        
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SLEEP));
    }

    // Execute a cycle start by starting the stepper interrupt.
    if (rt_exec & EXEC_CYCLE_START) {
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL))) {
        // Cycle start only when IDLE or when a hold is complete and ready to resume.
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
          sys.step_control = STEP_CONTROL_NORMAL_OP; // Restore step control to normal operation
          if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
            sys.suspend = SUSPEND_DISABLE; // Break suspend state.
            sys.state = STATE_CYCLE;
            st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
            st_wake_up();
          } else { // Otherwise, do nothing. Set and resume IDLE state.
            sys.suspend = SUSPEND_DISABLE; // Break suspend state.
            sys.state = STATE_IDLE;
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      if ((sys.state & (STATE_HOLD|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
        // Motion complete. Includes CYCLE/JOG/HOMING states and jog cancel/motion cancel/soft limit events.
        if (sys.suspend & SUSPEND_JOG_CANCEL) {
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        sys.suspend = SUSPEND_DISABLE;
        sys.state = STATE_IDLE;
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  // Execute feed/rapid overrides.
  rt_exec = sys_rt_exec_motion_override; // Copy volatile
  if (rt_exec) {
    system_clear_exec_motion_overrides(); // Clear all motion override flags.

    uint8_t new_f_override =  sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min(new_f_override,MAX_FEED_RATE_OVERRIDE);
    new_f_override = max(new_f_override,MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; // Set to report change immediately
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif

  // Reload step segment buffer
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


// Handles Grbl system suspend procedures, such as feed hold.
static void protocol_exec_rt_suspend()
{
  plan_block_t *block = plan_get_current_block();
  uint8_t restore_condition;
  if (block == NULL) { restore_condition = gc_state.modal.spindle; }
  else { restore_condition = block->condition; }

  while (sys.suspend) {

    if (sys.abort) { return; }

    // Block until initial hold is complete and the machine has stopped motion.
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

      // Sleep state handling.
      if (sys.state == STATE_SLEEP) {
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {
          spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;
        } else {
          report_feedback_message(MESSAGE_SLEEP_MODE);
          spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
          st_go_idle(); // Disable steppers
          while (!(sys.abort)) { protocol_exec_rt_system(); } // Do nothing until reset.
          return; // Abort received. Return to re-initialize.
        }
      }

    }

    protocol_exec_rt_system();

  }
}
