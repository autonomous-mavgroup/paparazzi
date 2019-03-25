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
#include "modules/group10/group10_bla.h"
#include "modules/group10/BLA.h

#ifndef GROUP10_BLA_FPS
#define GROUP10_BLA_FPS 0
#endif
PRINT_CONFIG_VAR(GROUP10_BLA_FPS)

struct image_t *group10_opencv_bla(struct image_t *img);
int *group10_opencv_bla(struct image_t *img)
{
    if (img->type == IMAGE_YUV422)
    {
        BLA((char *) img->buf, img->w, img->h);
    }
}

void group10_bla_init(void)
{
    cv_add_to_device(&OPENCVDEMO_CAMERA, group10_opencv_bla, GROUP10_BLA_FPS);
}


