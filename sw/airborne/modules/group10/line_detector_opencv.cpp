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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "line_detector_opencv.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <vector>
#include <string.h>
std::vector<float> diff(vector<cv::Point2i> points);


void grayscale_opencv_to_yuv422(cv::Mat image, char *img, int width, int height)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 1);

  int n_rows = image.rows;
  int n_cols = image.cols;

  // If the image is one block in memory we can iterate over it all at once!
  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  // Iterate over the image, setting only the Y value
  // and setting U and V to 127
  int i, j;
  uchar *p;
  int index_img = 0;
  for (i = 0; i < n_rows; ++i) {
    p = image.ptr<uchar>(i);
    for (j = 0; j < n_cols; j++) {
      img[index_img++] = 127;
      img[index_img++] = p[j];


    }
  }
}

// float[3] coef(vector<cv::Vec2i> points, &float[3] coefficients )
// {
//   points_size = points.size();
//   float sum_x, sum_y, sum_xy, sum_x2, sum_x2y, sum_x3, sum_x4;
//   float S_xx, S_xy, S_xx2, S_x2y, Sx2x2, avg_y, avg_x;
//   sum_y =0;
//   sum_x = 0;
//   sum_xy = 0;
//   sum_x2 = 0;
//   sum_x2y =0;
//   sum_x3 = 0;
//   sum_x4 = 0;
//   S_xx = 0;
//   S_xy = 0;
//   S_xx2 = 0; 
//   S_x2y = 0;
//   S_x2x2 = 0; 
//   avg_y = 0;
//   avg_x = 0;
  
//   for(int i = 0; i<points_size; i++)
//   {
//     float x2 = points[i].x * points[i].y;
//     sum_y   += points[i].x;
//     sum_x   += points[i].y;
//     sum_xy  += points[i].y * points[i].x;
//     sum_x2  += x2;
//     sum_x2y += x2 * points[i].x;
//     sum_x3  += points[i].y * x2;
//     sum_x4  += x2 * x2;
//   }

//   avg_y = sum_y / points_size;
//   avg_x = sum_x / points_size;

//   S_xx   = sum_x2  - sum_x  * avg_x;
//   S_xy   = sum_xy  - sum_x  * avg_y;
//   S_xx2  = sum_x3  - sum_x2 * avg_x;
//   S_x2y  = sum_x2y - sum_x2 * avg_y;
//   S_x2x2 = sum_x4  - sum_x2 * sum_x2 / no_points;
//   scaler = 1.0f / (S_xx* S_x2x2 - S_xx2*S_xx2);

//   coefficients[2] = (S_x2y * S_xx - S_xy * S_xx2) * scaler;
//   coefficients[1] = (S_xy * S_x2x2 - S_x2y * S_xx2) * scaler;
//   coefficients[0] = (sum_y - coefficients[1] * sum_x - coefficients[2] * sum_x2) / no_points;

// }


std::vector<float> diff(vector<cv::Point2i> points)
{
  std::vector<float> res;
  float items[3];
  int point_size = points.size();
  for(int i = 0; i < point_size-1; i++)
  {
    items[0] = points[i].y;
    items[1] = points[i].x;
    items[2] = std::abs(points[i+1].x - points[i].x)/(points[i+1].y - points[i].y); 
    res.push_back(items[0]);
    res.push_back(items[1]);
    res.push_back(items[2]);
  }

  for(int i = 0; i < point_size-2; i++)
  {
    res[i*3+2] = abs(res[(i+1)*3+2] - res[i*3+2]);
  }
  return res;
}

int detect_line_opencv(char *img, int width, int height, char *out, settings set_conf)
{
  // Create a new image, using the original bebop image.
  cv::Mat image(height, width, CV_8UC2, img);
  cv::Mat mask;
  cv::Mat maskgray;
  cv::Mat edges;
  cv::Mat res(height, width, CV_8UC2);
  // Image already in YUV format

  cv::Scalar lower = cv::Scalar(  set_conf.min_u,   set_conf.min_v,   set_conf.min_y);
  cv::Scalar upper = cv::Scalar(  set_conf.max_u,  set_conf.max_v,  set_conf.max_y);

  // cv::Scalar lower = cv::Scalar(0, 2, 70);
  // cv::Scalar upper = cv::Scalar(181,52,111);
  
  cv::inRange(image, lower, upper, mask);
  // cv::cvtColor(mask, maskgray, CV_GRAY2YUV_Y422)
  grayscale_opencv_to_yuv422(mask, out, width, height);

  // memcpy(out, maskgray.data, 2*width*height);
  cv::bitwise_and(image, image, image, mask);

  //cv::blur(mask, mask, cv::Size(2,2));
  cv::Canny(mask, edges, 300, 400);

  float kernel_data[9] = {-1.0f, 2.0f, -1.0f, -1.0f, 2.0f, -1.0f, -1.0f, 2.0f, -1.0f};

  cv::Mat kernel = cv::Mat(3,3,CV_32F, kernel_data);

  cv::filter2D(mask, edges, -1, kernel);

  cv::Size edges_size = edges.size();

  int no_lines = 100;

  vector<cv::Point2i> points;

  int limit = (int)edges_size.height*0.4;
  bool found_points = false;
  for(int i = edges_size.height-1; i > limit; i--)
  {
    for(int j = 0; j<edges_size.width; j++)
    {
      if(edges.at<int>(i,j) > 0)
      {
        int sum = 0;
        for(int k = 0; k<4; k++)
        {
          sum += mask.at<int>(i-k,j);
        }
        if(sum == 0)
        {
          points.push_back(cv::Point2i(i,j));
          found_points = true;
        }
      }
    }
  }

  vector<cv::Point2i> positions;
  if(found_points == true)
  {
    std::vector<float> diff_points = diff(points);
    int diff_points_size = diff_points.size() / 3;

    for(int i = 0; i < diff_points_size; i++)
    {
      if(diff_points[i*3+1])
      {
          int x = int(diff_points[i*3+0]);
          int y = int(diff_points[i*3+1]);
          positions.push_back(cv::Point2i(x,y));
      }
    }
  }

  float FOV_x = 120.0f/180.0f*3.14159f;
  float FOV_y = 120.0f/180.0f*3.14159f;
  float alt = 1.5;
  vector<cv::Point2f> fin;
  for(int i = 0; i<positions.size(); i++)
  {
    float theta = (positions[i].y - height/2.0f)*(FOV_y/height);
    float psi = (positions[i].x - width/2.0f)*(FOV_x/width);
    float d = alt/(atan(theta));
    fin.push_back(cv::Point2f(d*sin(psi),d*cos(psi)));
  }

  return 0;
}
