/*
  job_state.h - Save/resume job state to EEPROM for power loss recovery
  Part of Grbl - Pen Plotter Edition

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#ifndef job_state_h
#define job_state_h

#ifdef ENABLE_JOB_STATE_SAVE

// Each EEPROM slot stores:
//   float x (4 bytes) + float y (4 bytes) + uint8_t sequence (1 byte) + uint8_t checksum (1 byte) = 10 bytes
#define JOB_STATE_SLOT_SIZE 10

// Initialize job state module (find current slot on boot)
void job_state_init();

// Called periodically from mc_line. Increments counter and saves when interval is reached.
void job_state_tick(float *position);

// Report saved position on startup (prints [RESUME:X...,Y...] if valid)
void job_state_report();

// Clear all saved job state slots (called by $RST=J)
void job_state_clear();

#endif // ENABLE_JOB_STATE_SAVE

#endif // job_state_h
