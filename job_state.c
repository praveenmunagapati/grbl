/*
  job_state.c - Save/resume job state to EEPROM for power loss recovery
  Part of Grbl - Pen Plotter Edition

  Uses a wear-leveled ring buffer of EEPROM slots. Each slot stores the X/Y
  machine position with a sequence counter. On boot, the slot with the highest
  sequence number is found and its position is reported for resume.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "grbl.h"

#ifdef ENABLE_JOB_STATE_SAVE

static uint8_t job_state_slot_idx;    // Current ring buffer slot index
static uint8_t job_state_seq;         // Current sequence counter
static uint16_t job_state_counter;    // Block counter between saves
static uint8_t job_state_has_data;    // Flag: valid data found on boot

static float job_state_saved_x;       // Cached X from last boot read
static float job_state_saved_y;       // Cached Y from last boot read


// Read a slot from EEPROM. Returns 1 if checksum valid, 0 if not.
static uint8_t job_state_read_slot(uint8_t slot, float *x, float *y, uint8_t *seq)
{
  unsigned int addr = EEPROM_ADDR_JOB_STATE + (slot * JOB_STATE_SLOT_SIZE);
  
  // Read raw bytes
  char buf[JOB_STATE_SLOT_SIZE];
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < JOB_STATE_SLOT_SIZE - 1; i++) {
    buf[i] = eeprom_get_char(addr + i);
    checksum = (checksum << 1) | (checksum >> 7); // Rotate left
    checksum += buf[i];
  }
  uint8_t stored_checksum = eeprom_get_char(addr + JOB_STATE_SLOT_SIZE - 1);
  
  if (checksum != stored_checksum) { return 0; }
  
  // Unpack
  memcpy(x, &buf[0], sizeof(float));
  memcpy(y, &buf[4], sizeof(float));
  *seq = buf[8];
  
  return 1;
}


// Write a slot to EEPROM with checksum.
static void job_state_write_slot(uint8_t slot, float x, float y, uint8_t seq)
{
  unsigned int addr = EEPROM_ADDR_JOB_STATE + (slot * JOB_STATE_SLOT_SIZE);
  
  char buf[JOB_STATE_SLOT_SIZE];
  memcpy(&buf[0], &x, sizeof(float));
  memcpy(&buf[4], &y, sizeof(float));
  buf[8] = seq;
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < JOB_STATE_SLOT_SIZE - 1; i++) {
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += buf[i];
  }
  buf[JOB_STATE_SLOT_SIZE - 1] = checksum;
  
  // Write to EEPROM
  for (uint8_t i = 0; i < JOB_STATE_SLOT_SIZE; i++) {
    eeprom_put_char(addr + i, buf[i]);
  }
}


// Initialize: scan all slots to find the most recent valid one.
void job_state_init()
{
  job_state_counter = 0;
  job_state_has_data = 0;
  job_state_slot_idx = 0;
  job_state_seq = 0;
  
  uint8_t best_seq = 0;
  uint8_t found = 0;
  
  for (uint8_t i = 0; i < JOB_STATE_NUM_SLOTS; i++) {
    float x, y;
    uint8_t seq;
    if (job_state_read_slot(i, &x, &y, &seq)) {
      // Use simple "nearest higher" sequence logic.
      // The most recently written slot has the highest sequence number.
      if (!found || ((int8_t)(seq - best_seq) > 0)) {
        best_seq = seq;
        job_state_slot_idx = i;
        job_state_saved_x = x;
        job_state_saved_y = y;
        found = 1;
      }
    }
  }
  
  if (found) {
    job_state_has_data = 1;
    job_state_seq = best_seq;
    // Advance to next slot for future writes
    job_state_slot_idx = (job_state_slot_idx + 1) % JOB_STATE_NUM_SLOTS;
  }
}


// Called from mc_line on every motion block. Saves position at configured interval.
void job_state_tick(float *position)
{
  job_state_counter++;
  if (job_state_counter >= JOB_STATE_SAVE_INTERVAL) {
    job_state_counter = 0;
    job_state_seq++;
    
    job_state_write_slot(job_state_slot_idx, position[X_AXIS], position[Y_AXIS], job_state_seq);
    
    // Advance ring buffer
    job_state_slot_idx = (job_state_slot_idx + 1) % JOB_STATE_NUM_SLOTS;
  }
}


// Report saved position on startup.
void job_state_report()
{
  if (job_state_has_data) {
    printPgmString(PSTR("[RESUME:X"));
    printFloat(job_state_saved_x, N_DECIMAL_COORDVALUE_MM);
    printPgmString(PSTR(",Y"));
    printFloat(job_state_saved_y, N_DECIMAL_COORDVALUE_MM);
    printPgmString(PSTR("]\r\n"));
  }
}


// Clear all job state EEPROM slots.
void job_state_clear()
{
  for (uint8_t i = 0; i < JOB_STATE_NUM_SLOTS; i++) {
    unsigned int addr = EEPROM_ADDR_JOB_STATE + (i * JOB_STATE_SLOT_SIZE);
    for (uint8_t j = 0; j < JOB_STATE_SLOT_SIZE; j++) {
      eeprom_put_char(addr + j, 0xFF);
    }
  }
  job_state_has_data = 0;
  job_state_counter = 0;
  job_state_seq = 0;
  job_state_slot_idx = 0;
}


#endif // ENABLE_JOB_STATE_SAVE
