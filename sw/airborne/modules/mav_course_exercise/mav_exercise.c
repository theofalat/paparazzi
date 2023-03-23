/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"
#include "firmwares/rotorcraft/autopilot_guided.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance.h"
#include <stdio.h>
#include <time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Proportional gain for the yaw rate control
# ifndef KPI
# define KPI 0.0025
# endif

// Derivative gain for the yaw rate control
# ifndef KDI
# define KDI 0.000001
# endif

// 'Damping' gain (unused)
# ifndef KYD
# define KYD 1000
# endif

// Yaw rate magnitude limit
# ifndef YI
# define YI 1.1
# endif

// Nominal velocity command mangitude
# ifndef VI
# define VI 0.4
# endif

// (unused)
# ifndef DIVI
# define DIVI 0.0001f
# endif

// Optical flow difference threshold
# ifndef OFDI
# define OFDI 350
# endif

// Threshold for the 'green count' of the floor
# ifndef GRDI
# define GRDI 1000
# endif

// Divergence threshold (unused)
# ifndef DVDI
# define DVDI 2000
# endif

// How many times divergence has to be above threshold to trigger obstacle avoidace (unused)
# ifndef RBST
# define RBST 6
# endif

// States for our state machine
enum navigation_state_t {
  SAFE,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  GOBACK,
  TURN
};

// Define and initialise global variables
enum navigation_state_t navigation_state = SAFE; // Start in safe state
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)

float div_thresh = 0.f;                 // From original file
float heading_increment = 0.f;

float div_1 = 0.f;                      // Divergence
float divergence_thresh = DVDI;         // Divergence threshold
double Kp = KPI;                        // Proportional gain for yaw rate control
double Kd = KDI;                        // Derivative gain
double Kyd = KYD;                       // 'Damping' gain
float yaw_rate = 0;                     // yaw rate that we will set
float green_thresh = GRDI;              // Threshold for the count of green pixels on the floor
float of_diff_thresh = OFDI;            // Trheshold for the OF difference
double of_diff;                          // difference in optical flow between right and left side
double of_diff_prev = 0;                // OF difference in previous loop
float yaw_thresh = YI;                  // Threshold (limit) for the commnaded yaw rate
float dr_vel = VI;                      // Forward velocity to be commanded
int count_backwards=0;                  // Counter for the GOBACK state
int count_oob=0;                        // Counter for OOB state
int robust = RBST;                      // Threshold for counter below for the divergence
int count_robust = 0;                 // Counter to keep track of divergence exceeding set threshold

// For our OF diff message
#ifndef OF_DIFF_DIV_ID
#define OF_DIFF_DIV_ID ABI_BROADCAST
#endif

// For floor count message
#ifndef FLOOR_VISUAL_DETECTION_ID
#define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

// Event and callback for floor green pixel count message
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((un45used)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

// Event and callback for OF diff and div message
static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id, double of_diff_value, float div_value) {
  div_1 = div_value;
  of_diff = of_diff_value;
}


void mav_exercise_init(void) {
  // bind callbacks
  AbiBindMsgOF_DIFF_DIV(OF_DIFF_DIV_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

void mav_exercise_periodic(void) {
  // Only evaluate our state machine if we are flying (in guided mode)

  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SAFE;
    return;
  }

  // Printing of useful numbers
  PRINT("OF difference: %f \n", of_diff);
  PRINT("OF difference prev: %f \n", of_diff_prev);
  PRINT("Yaw rate: %f \n", stateGetBodyRates_f()->r);
  PRINT("Divergence value is: %f \n",div_1);
  PRINT("Green count value is: %d \n",floor_count);

  switch (navigation_state) {
    // Safe state, when there is no need to avoid anything
    case SAFE:
      PRINT("SAFE STATE \n");
      // First check, if not inside, set state to OOB
      if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y ))
      {
        navigation_state = OUT_OF_BOUNDS;
      }
      // If divergence is above threshold several times, set state to GOBACK
      else if(div_1 > divergence_thresh)
      {
        count_robust++;
        if(count_robust >= robust)
        {
          navigation_state = GOBACK;
        }
      }
      // Also check green pixel count on floor, if below threshold, also set state to GOBACK
      else if(floor_count < green_thresh)
      {
    	  navigation_state = GOBACK;
      }
      // Otherwise if optical flow difference is above threshold, switch to turning state
      else if(fabs(of_diff)>of_diff_thresh)
      {
          count_robust = 0;
          navigation_state = TURN;
      }
      // Otherwise just keep going forward
      else
      {
          count_robust = 0;
          PRINT("NO ROTATION \n");
          guidance_h_set_guided_body_vel(dr_vel, 0);
          guidance_h_set_guided_heading_rate(0);
      }

      break;
    // Turn state, when turning is necessary to avoid obstacles
    case TURN:
      PRINT("TURN STATE \n");
      // OOB check again first
      if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y ))
      {
        navigation_state = OUT_OF_BOUNDS;
      }
      // If optical flow difference is below threshold, go back to safe state
      if(fabs(of_diff)-fabs(Kyd * stateGetBodyRates_f()->r) < of_diff_thresh)
      {
        navigation_state = SAFE;
      }
      // Same check as before for green pixel count on floor
      else if(floor_count < green_thresh)
      {
    	  navigation_state = GOBACK;
      }
      // Setting yaw rate to gain * optical flow difference, clipping at maximum and minimum yaw values
      yaw_rate =  -Kp * of_diff + Kd * (of_diff - of_diff_prev);
      if(yaw_rate > yaw_thresh){yaw_rate = yaw_thresh;}
      else if(yaw_rate < -yaw_thresh){yaw_rate = -yaw_thresh;}
      guidance_h_set_guided_heading_rate(yaw_rate);
      guidance_h_set_guided_body_vel(dr_vel, 0);
      break;

    case GOBACK:
      PRINT("GOBACK STATE \n");
      if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y )) {
          navigation_state = OUT_OF_BOUNDS;
        }
      // Go backwards for two counts
      else if(count_backwards<=4)
      {
          guidance_h_set_guided_body_vel(-0.1, 0);
          guidance_h_set_guided_heading_rate(0);
          count_backwards++;
          PRINT("GO BACK \n");
      }
      // Then switch to OOB state, reset counter
      else
      {
        navigation_state = OUT_OF_BOUNDS;
        count_backwards=0;
      }
      break;

    case OUT_OF_BOUNDS:
      PRINT("OOB STATE \n");
      // Always stopping
      guidance_h_set_guided_body_vel(0, 0);
      // On 'first' loop, reverse a bit and set target heading to current + 160 deg
      if(count_oob == 0)
      {
          guidance_h_set_guided_body_vel(-dr_vel, 0);
      	  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + RadOfDeg(160));
      }
      // Essentially a counter to make it wait so that it doesn't immediately go to next state
      if(count_oob > 6)
      {
    	  navigation_state = REENTER_ARENA;
    	  count_oob=0;
      }
      else
      {
    	  // Increase counter
    	  count_oob++;
      }
      break;

    case REENTER_ARENA:
      PRINT("REENTER STATE \n");
      // Keep going until back inside, then switch back to safe state

    	  guidance_h_set_guided_body_vel(dr_vel, 0);
          // If green count very low, likely facing wrong direction even after OOB turn, so turn a bit more
    	  if (floor_count < green_thresh)
    	  {
          	  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + RadOfDeg(60));
    	  }
    	  // Once inside again, go back to safe state
    	  else if(InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y ))
    	  {
    		  navigation_state = SAFE;
    	  }

      break;

    default:
      break;



  }
  // Update previous value of OF diff for next loop
  of_diff_prev = of_diff;
  return;
}