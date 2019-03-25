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
static struct image_t *send_numbers(struct image_t *img);

static pthread_mutex_t mutex;
struct color_object_t {
    int32_t x_c;
    int32_t y_c;
    uint32_t color_count;
    bool updated;
};
struct color_object_t global_filters[2];

void group10_bla_init(void)
{
    memset(global_filters, 0, 2*sizeof(struct color_object_t));
    pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
    #ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1

#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, send_numbers, COLOR_OBJECT_DETECTOR_FPS1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
    #ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2);
#endif
}

static struct image_t *send_numbers(struct image_t *img){
    pthread_mutex_lock(&mutex);
    global_filters[0].color_count = 0;
    global_filters[0].x_c = 0;
    global_filters[0].y_c = 0;
    global_filters[0].updated = true;
    pthread_mutex_unlock(&mutex);
}

void group10_bla_peridic(void)
{
    static struct color_object_t local_filters[2];
    pthread_mutex_lock(&mutex);
    memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
    pthread_mutex_unlock(&mutex);

    if(local_filters[0].updated){
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
                                   0, 0, local_filters[0].color_count, 0);
        local_filters[0].updated = false;
    }
    if(local_filters[1].updated){
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
                                   0, 0, local_filters[1].color_count, 1);
        local_filters[1].updated = false;
    }
}

