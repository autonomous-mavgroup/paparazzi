/*
 * Copyright (C) matteo
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
 * @file "modules/group10/group10_bla.c"
 * @author matteo
 * 
 */
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"
#include "modules/computer_vision/cv.h"
#include "modules/group10/group10_bla.h"
#include "modules/group10/BLA.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef GROUP10_BLA_FPS
#define GROUP10_BLA_FPS 0
#endif
PRINT_CONFIG_VAR(GROUP10_BLA_FPS)
VERBOSE_PRINT("module group 10")
static struct image_t *send_numbers(struct image_t *img);

static pthread_mutex_t mutex;
struct blaData {
    int32_t heading;
    bool reached_edge;
    bool updated;
};
struct blaData* bla_data;

void group10_bla_init(void)
{
    VERBOSE_PRINT("INITIALIZING CV MODULE");
    memset(bla_data, 0, sizeof(struct blaData));
    pthread_mutex_init(&mutex, NULL);
#ifdef BLA_CAMERA1
    #ifdef BLA_CAMERA1

#endif
  cv_add_to_device(&BLA_CAMERA1, send_numbers, GROUP10_BLA_FPS);
#endif

}

static struct image_t *send_numbers(struct image_t *img){
    pthread_mutex_lock(&mutex);
    bla_data->heading = 10;
    bla_data->reached_edge = false;
    bla_data->updated = true;
    pthread_mutex_unlock(&mutex);
}

void group10_bla_periodic(void)
{
    static struct blaData* local_bla_data;
    pthread_mutex_lock(&mutex);
    memcpy(local_bla_data, bla_data, sizeof(struct blaData));
    pthread_mutex_unlock(&mutex);

    if(local_bla_data->updated){
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, 0, 0,
                                   0, 0, local_bla_data->heading, 0);
        local_bla_data->updated = false;
    }
}

