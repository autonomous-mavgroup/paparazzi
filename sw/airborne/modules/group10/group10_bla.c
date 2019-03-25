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

#ifndef GROUP10_BLA_FPS
#define GROUP10_BLA_FPS 0
#endif
PRINT_CONFIG_VAR(GROUP10_BLA_FPS)
static struct image_t *main_func(struct image_t *img);

static pthread_mutex_t mutex;
struct bla_object_t {
    int32_t heading;
    bool reached_edge;
    bool updated;
};
struct bla_object_t bla_data[2];

void group10_bla_init(void)
{
    memset(bla_data, 0, 2*sizeof(struct bla_object_t));
    pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
    #ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1

#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, main_func, COLOR_OBJECT_DETECTOR_FPS1);
#endif
}

static struct image_t *main_func(struct image_t *img){
    pthread_mutex_lock(&mutex);
    bla_data[0].heading = 10;
    if (bla_data[0].reached_edge == 0) {
        bla_data[0].reached_edge = 1;
    } else {
    bla_data[0].reached_edge = 0;
    }
    bla_data[0].updated = true;
    pthread_mutex_unlock(&mutex);
}

void group10_bla_periodic(void)
{
    static struct bla_object_t local_filters[2];
    pthread_mutex_lock(&mutex);
    memcpy(local_filters, bla_data, 2*sizeof(struct bla_object_t));
    pthread_mutex_unlock(&mutex);

    if(local_filters[0].updated){
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, 0, 0,
                                   0, 0, local_filters[0].heading, local_filters[0].reached_edge);
        local_filters[0].updated = false;
    }
}

