/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file actuators_asctec.c
 *  Actuators driver for Asctec motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_asctec.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#define ASCTEC_MIN_CMD -100
#define ASCTEC_MAX_CMD 100

#define ASCTEC_MIN_THROTTLE 0
#define ASCTEC_MAX_THROTTLE 200

struct ActuatorsAsctec actuators_asctec;

uint32_t actuators_delay_time;
bool_t   actuators_delay_done;

void actuators_asctec_init(void) {
  actuators_asctec.cmd = NONE;
  actuators_asctec.cur_addr = FRONT;
  actuators_asctec.new_addr = FRONT;
  actuators_asctec.i2c_trans.status = I2CTransSuccess;
  actuators_asctec.i2c_trans.type = I2CTransTx;
  actuators_asctec.i2c_trans.slave_addr = 0x02;
#ifdef ACTUATORS_ASCTEC_V2_PROTOCOL
  actuators_asctec.i2c_trans.len_w = 5;
#else
  actuators_asctec.i2c_trans.len_w = 4;
#endif
  actuators_asctec.nb_err = 0;

#if defined ACTUATORS_START_DELAY && ! defined SITL
  actuators_delay_done = FALSE;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = TRUE;
  actuators_delay_time = 0;
#endif

}

#ifndef ACTUATORS_ASCTEC_V2_PROTOCOL
void actuators_asctec_set(bool_t motors_on) {
#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) return;
    else actuators_delay_done = TRUE;
  }
#endif

  switch (actuators_asctec.i2c_trans.status) {
    case I2CTransFailed:
      actuators_asctec.nb_err++;
      actuators_asctec.i2c_trans.status = I2CTransDone;
      break;
    case I2CTransSuccess:
    case I2CTransDone:
      actuators_asctec.i2c_trans.status = I2CTransDone;
      break;
    default:
      actuators_asctec.nb_err++;
      return;
  }

#ifdef KILL_MOTORS
  actuators_asctec.cmds[PITCH]  = 0;
  actuators_asctec.cmds[ROLL]   = 0;
  actuators_asctec.cmds[YAW]    = 0;
  actuators_asctec.cmds[THRUST] = 0;
#else /* ! KILL_MOTORS */
  Bound(actuators_asctec.cmds[PITCH],ASCTEC_MIN_CMD, ASCTEC_MAX_CMD);
  Bound(actuators_asctec.cmds[ROLL], ASCTEC_MIN_CMD, ASCTEC_MAX_CMD);
  Bound(actuators_asctec.cmds[YAW],  ASCTEC_MIN_CMD, ASCTEC_MAX_CMD);
  if (motors_on) {
    Bound(actuators_asctec.cmds[THRUST],  ASCTEC_MIN_THROTTLE + 1, ASCTEC_MAX_THROTTLE);
  }
  else
    actuators_asctec.cmds[THRUST] = 0;
#endif /* KILL_MOTORS  */

  switch (actuators_asctec.cmd) {
  case TEST:
    actuators_asctec.i2c_trans.buf[0] = 251;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = 0;
    actuators_asctec.i2c_trans.buf[3] = 231 + actuators_asctec.cur_addr;
    break;
  case REVERSE:
    actuators_asctec.i2c_trans.buf[0] = 254;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = 0;
    actuators_asctec.i2c_trans.buf[3] = 234 + actuators_asctec.cur_addr;
    break;
  case SET_ADDR:
    actuators_asctec.i2c_trans.buf[0] = 250;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = actuators_asctec.new_addr;
    actuators_asctec.i2c_trans.buf[3] = 230 + actuators_asctec.cur_addr + actuators_asctec.new_addr;
    actuators_asctec.cur_addr = actuators_asctec.new_addr;
    break;
  case NONE:
    actuators_asctec.i2c_trans.buf[0] = 100 - actuators_asctec.cmds[PITCH];
    actuators_asctec.i2c_trans.buf[1] = 100 + actuators_asctec.cmds[ROLL];
    actuators_asctec.i2c_trans.buf[2] = 100 - actuators_asctec.cmds[YAW];
    actuators_asctec.i2c_trans.buf[3] = actuators_asctec.cmds[THRUST];
    break;
  default:
    break;
  }
  actuators_asctec.cmd = NONE;

  actuators_asctec.i2c_trans.type = I2CTransTx;   // Can be reset I2C driver
  i2c_submit(&ACTUATORS_ASCTEC_DEVICE, &actuators_asctec.i2c_trans);

}
#else /* ! ACTUATORS_ASCTEC_V2_PROTOCOL */
void actuators_asctec_set(bool_t motors_on) {
#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) {
#ifdef USE_I2C_ACTUATORS_REBOOT_HACK
      //Lisa-L with Asctech v2 motors only start after reflashing when a bus error was sensed on stm32-i2c.
      //multiple re-init solves the problem.
      i2c1_init();
#endif
      return;
    }
    else actuators_delay_done = TRUE;
  }
#endif

  switch (actuators_asctec.i2c_trans.status) {
    case I2CTransFailed:
      actuators_asctec.nb_err++;
      actuators_asctec.i2c_trans.status = I2CTransDone;
      break;
    case I2CTransSuccess:
    case I2CTransDone:
      actuators_asctec.i2c_trans.status = I2CTransDone;
      break;
    default:
      actuators_asctec.nb_err++;
      return;
  }

#ifdef KILL_MOTORS
  actuators_asctec.i2c_trans.buf[0] = 0;
  actuators_asctec.i2c_trans.buf[1] = 0;
  actuators_asctec.i2c_trans.buf[2] = 0;
  actuators_asctec.i2c_trans.buf[3] = 0;
  actuators_asctec.i2c_trans.buf[4] = 0xAA;
#else
  switch (actuators_asctec.cmd) {
  case TEST:
    actuators_asctec.i2c_trans.buf[0] = 251;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = 0;
    actuators_asctec.i2c_trans.buf[3] = 231 + actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.len_w = 4;
    break;
  case REVERSE:
    actuators_asctec.i2c_trans.buf[0] = 254;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = 0;
    actuators_asctec.i2c_trans.buf[3] = 234 + actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.len_w = 4;
    break;
  case SET_ADDR:
    actuators_asctec.i2c_trans.buf[0] = 250;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = actuators_asctec.new_addr;
    actuators_asctec.i2c_trans.buf[3] = 230 + actuators_asctec.cur_addr + actuators_asctec.new_addr;
    actuators_asctec.cur_addr = actuators_asctec.new_addr;
    actuators_asctec.i2c_trans.len_w = 4;
    break;
  case NONE:
    actuators_asctec.i2c_trans.buf[0] = actuators_asctec.cmds[SERVO_FRONT];
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cmds[SERVO_BACK];
    actuators_asctec.i2c_trans.buf[2] = actuators_asctec.cmds[SERVO_LEFT];
    actuators_asctec.i2c_trans.buf[3] = actuators_asctec.cmds[SERVO_RIGHT];
    actuators_asctec.i2c_trans.buf[4] = 0xAA + actuators_asctec.i2c_trans.buf[0] + actuators_asctec.i2c_trans.buf[1] +
      actuators_asctec.i2c_trans.buf[2] + actuators_asctec.i2c_trans.buf[3];
    actuators_asctec.i2c_trans.len_w = 5;
    break;
  default:
    break;
  }
  actuators_asctec.cmd = NONE;
#endif

  actuators_asctec.i2c_trans.type = I2CTransTx;   // Can be reset I2C driver
  i2c_submit(&ACTUATORS_ASCTEC_DEVICE, &actuators_asctec.i2c_trans);

}
#endif /* ACTUATORS_ASCTEC_V2_PROTOCOL */
