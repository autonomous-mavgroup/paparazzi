/*
 * Copyright (C) RR
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/group10/obstacle_avoider.c"
 * @author RR
 * main module for course
 */

#include "modules/group10/obstacle_avoider.h"
#include "subsystems/abi.h"
#include "modules/computer_vision/cv.h"

// void obstacle_detected() {}

void obstacle_avoider_init()
{

  AbiBindMsgLINE_DETECTED(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

}

void obstacle_avoider_loop()
{

}


