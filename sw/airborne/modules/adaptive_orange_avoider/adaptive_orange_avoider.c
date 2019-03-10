/*
 * Copyright (C) Lorenzo Terenzi
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/adaptive_orange_avoider/adaptive_orange_avoider.c"
 * @author Lorenzo Terenzi
 * testing if changes in avoid-orange are effective
 */

#include "modules/adaptive_orange_avoider/adaptive_orange_avoider.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS
};



#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    color_count = quality;
}

void adaptative_orange_avoider_init(void)
{
    // Initialise random values
    srand(time(NULL));
    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

void adaptative_orange_avoider_periodic {
        // define settings
        float oa_color_count_frac = 0.18f;

// define and initialise global variables
        enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
        int32_t color_count = 0;               // orange color count from color filter for obstacle detection
        int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
        float heading_increment = 5.f;          // heading angle increment [deg]
        float maxDistance = 2.25;               // max waypoint displacement [m]

        const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
        // only evaluate our state machine if we are flying
        if(!autopilot_in_flight()){
            return;
        }

        // compute current color thresholds
        int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

        VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count, navigation_state);

        // update our safe confidence using color threshold
        if(color_count < color_count_threshold){
            obstacle_free_confidence++;
        } else {
            obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
        }
        Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

        float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

}


