/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/group10/line_detector.h"
#include "modules/group10/line_detector_opencv.h"
#include "modules/computer_vision/viewvideo.h"
#include "subsystems/abi.h"

#include <pthread.h>

#ifndef GROUP10_LINE_DETECTOR_FPS
#define GROUP10_LINE_DETECTOR_FPS 5       ///< Default FPS (zero means run at camera fps)
#endif
struct image_t out;
struct settings lsd;
static pthread_mutex_t mutex;
int control;

// Function
struct image_t *detect_line(struct image_t *img);
struct image_t *detect_line(struct image_t *img)
{
  int res = 0;
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    res = detect_line_opencv((char *) img->buf, img->w, img->h, (char *)out.buf, lsd);
    viewvideo_debug(&out);
  }

  pthread_mutex_lock(&mutex);
  control = res;
  pthread_mutex_unlock(&mutex);


  return NULL;
}

void group10_line_detector_periodic()
{
  int val = 0;
  pthread_mutex_lock(&mutex);
  val = control;
  pthread_mutex_unlock(&mutex);

  AbiSendMsgVISUAL_DETECTION(1, 0, 0, 0, 0, val, 0);
}



void group10_line_detector_init(void)
{
  control = -2; 
  pthread_mutex_init(&mutex, NULL);
  image_create(&out, 520, 60, IMAGE_YUV422);
  cv_add_to_device(&front_camera, detect_line, GROUP10_LINE_DETECTOR_FPS);
}

