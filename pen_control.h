/*
  pen_control.h - pen servo control methods
  Part of Grbl - Pen Plotter Edition

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef pen_control_h
#define pen_control_h

#define PEN_NO_SYNC false
#define PEN_FORCE_SYNC true

#define PEN_STATE_DISABLE  0  // Must be zero.
#define PEN_STATE_CW       bit(0)
#define PEN_STATE_CCW      bit(1)

// Initializes pen pin and pen servo.
void pen_init();

// Pen servo functions
void init_servo();
void pen_up();
void pen_down();
void pen_set_pressure(uint8_t tick);
void set_pen_pos();

// Returns current pen output state.
uint8_t pen_get_state();

// Called by g-code parser when setting pen state and requires a buffer sync.
#define pen_sync(state, rpm) _pen_sync(state)
void _pen_sync(uint8_t state);

// Sets pen running state with direction and enable.
#define pen_set_state(state, rpm) _pen_set_state(state)
void _pen_set_state(uint8_t state);

// Stop pen.
void pen_stop();

#endif
