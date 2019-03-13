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
 * @file "modules/orange_avoider_memory/orange_avoider_memory.h"
 * @author Lorenzo Terenzi
 * 
 */

#ifndef ORANGE_AVOIDER_MEMORY_H
#define ORANGE_AVOIDER_MEMORY_H

extern float oa_color_count_frac;

extern void orange_avoider_init();
extern void orange_avoider_periodic();

#endif

