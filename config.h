/*
  config.h - compile time configuration
  Part of Grbl - Pen Plotter Edition

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

// This file contains compile-time configurations for Grbl's internal system.
// Stripped down for pen plotter use only.

#ifndef config_h
#define config_h
#include "grbl.h" // For Arduino IDE compatibility.


// Define CPU pin map and default settings.
#define DEFAULTS_GENERIC
#define CPU_MAP_ATMEGA328P_SERVO // Arduino Uno CPU with pen servo

// Serial baud rate
#define BAUD_RATE 115200

// Define realtime command special characters.
#define CMD_RESET 0x18 // ctrl-x.
#define CMD_STATUS_REPORT '?'
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'

// Extended ASCII realtime commands
#define CMD_JOG_CANCEL  0x85
#ifdef DEBUG
  #define CMD_DEBUG_REPORT 0x86
#endif
#define CMD_FEED_OVR_RESET 0x90
#define CMD_FEED_OVR_COARSE_PLUS 0x91
#define CMD_FEED_OVR_COARSE_MINUS 0x92
#define CMD_FEED_OVR_FINE_PLUS  0x93
#define CMD_FEED_OVR_FINE_MINUS  0x94
#define CMD_RAPID_OVR_RESET 0x95
#define CMD_RAPID_OVR_MEDIUM 0x96
#define CMD_RAPID_OVR_LOW 0x97

// Homing init lock - forces homing before operation
// NOTE: Disabled because this machine has no limit switches.
// #define HOMING_INIT_LOCK

// Homing cycle - 2-axis sequential (X then Y)
#define HOMING_CYCLE_0 (1<<X_AXIS)  // First home X
#define HOMING_CYCLE_1 (1<<Y_AXIS)  // Then home Y

// Number of homing locate cycles
#define N_HOMING_LOCATE_CYCLE 1

// Number of startup blocks stored in EEPROM
#define N_STARTUP_LINE 0

// Floating decimal points for values
#define N_DECIMAL_COORDVALUE_INCH 4
#define N_DECIMAL_COORDVALUE_MM   3
#define N_DECIMAL_RATEVALUE_INCH  1
#define N_DECIMAL_RATEVALUE_MM    0
#define N_DECIMAL_SETTINGVALUE    3
#define N_DECIMAL_RPMVALUE        0

// Check limit switch states at init
// NOTE: Disabled - no limit switches installed.
// #define CHECK_LIMITS_AT_INIT

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// Debug mode
// #define DEBUG // Uncomment to enable.

// Feed rate override settings
#define DEFAULT_FEED_OVERRIDE           100
#define MAX_FEED_RATE_OVERRIDE          200
#define MIN_FEED_RATE_OVERRIDE           10
#define FEED_OVERRIDE_COARSE_INCREMENT   10
#define FEED_OVERRIDE_FINE_INCREMENT      1

#define DEFAULT_RAPID_OVERRIDE  100
#define RAPID_OVERRIDE_MEDIUM    50
#define RAPID_OVERRIDE_LOW       25

// Restore overrides after program end
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END

// Status report field configuration
// #define REPORT_FIELD_PIN_STATE
#define REPORT_FIELD_WORK_COORD_OFFSET
// #define REPORT_FIELD_OVERRIDES
// #define REPORT_FIELD_LINE_NUMBERS

// Status report refresh rates
#define REPORT_OVR_REFRESH_BUSY_COUNT 20
#define REPORT_OVR_REFRESH_IDLE_COUNT 10
#define REPORT_WCO_REFRESH_BUSY_COUNT 30
#define REPORT_WCO_REFRESH_IDLE_COUNT 10

// Acceleration ticks per second
#define ACCELERATION_TICKS_PER_SECOND 100

// Adaptive Multi-Axis Step Smoothing (AMASS)
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING

// Minimum planner junction speed
#define MINIMUM_JUNCTION_SPEED 0.0 // (mm/min)

// Minimum feed rate
#define MINIMUM_FEED_RATE 1.0 // (mm/min)

// Arc correction iterations
#define N_ARC_CORRECTION 12

// Arc full-circle epsilon
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)

// Dwell time step
#define DWELL_TIME_STEP 50 // milliseconds

// Planner and segment buffer sizes (uncomment to override defaults)
#define BLOCK_BUFFER_SIZE 12
// #define SEGMENT_BUFFER_SIZE 6

// Line buffer size
// #define LINE_BUFFER_SIZE 80

// Serial buffer sizes
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

// EEPROM restore commands
// #define ENABLE_RESTORE_EEPROM_WIPE_ALL
#define ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
// #define ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS

// Build info write command
// #define ENABLE_BUILD_INFO_WRITE_COMMAND

// Force buffer sync during EEPROM writes
#define FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE

// Force buffer sync during work coordinate offset changes
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE

// Job state save to EEPROM for resume after power loss
#define ENABLE_JOB_STATE_SAVE
#define JOB_STATE_SAVE_INTERVAL 50    // Save every N planner blocks
#define JOB_STATE_NUM_SLOTS 8         // Ring buffer slots for wear leveling


#endif
