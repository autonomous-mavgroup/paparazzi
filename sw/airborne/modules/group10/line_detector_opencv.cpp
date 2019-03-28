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


std::vector<float> diff(vector<cv::Point2i> points)
{
  std::vector<float> res;
  float items[3];
  int point_size = points.size();
  for(int i = 0; i < point_size-1; i++)
  {
    items[0] = points[i].x;
    items[1] = points[i].y;
    items[2] = std::abs((float)points[i+1].x - (float)points[i].x)/((float)points[i+1].y - (float)points[i].y); 
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

void uyvy_opencv_to_yuv_opencv(cv::Mat image, cv::Mat image_in, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec2b c = image_in.at<cv::Vec2b>(row, col);
      cv::Vec2b c_m1 = image_in.at<cv::Vec2b>(row, col);
      cv::Vec2b c_p1 = image_in.at<cv::Vec2b>(row, col);
      if (col > 0) {
        c_m1 = image_in.at<cv::Vec2b>(row, col - 1);
      }
      if (col < width) {
        c_p1 = image_in.at<cv::Vec2b>(row, col + 1);
      }
      image.at<cv::Vec3b>(row, col)[0] = c[1];
      image.at<cv::Vec3b>(row, col)[1] = col % 2 ?  c_m1[0] :  c[0];
      image.at<cv::Vec3b>(row, col)[2] = col % 2 ?  c[0] : c_p1[0];

    }
  }
}


void coloryuv_opencv_to_yuv422(cv::Mat image, char *img, int width, int height)
{
  int byte_index = 0;
  for(int r = 0; r < height; ++r) {
    for(int c = 0; c < width; ++c) {
      cv::Vec3b yuv = image.at<cv::Vec3b>(r, c);
      if((byte_index % 4) == 0) {
        img[byte_index++] = yuv.val[1]; // U
      } else {
        img[byte_index++] = yuv.val[2]; // V
      }
      img[byte_index++] = yuv.val[0]; // Y
    }
  }
}

int detect_line_opencv(char *img, int width, int height, char *out, settings set_conf)
{
  // Create a new image, using the original bebop image.
  cv::Mat image(height, width, CV_8UC2, img);
  cv::Mat imageyuv(height, width, CV_8UC3);
  cv::Mat cropped;
  cv::Mat mask;
  cv::Mat maskgray;
  cv::Mat edges;
  cv::Mat res(height, width, CV_8UC2);
  // Image already in YUV format

  float threshold = 2;
  float obs_threshold = 6;
  float total_threshold = 20;

  uyvy_opencv_to_yuv_opencv(imageyuv, image, 240, 520);;

  // cv::Scalar lower = cv::Scalar((int)set_conf.min_y, (int)set_conf.min_u,  (int)set_conf.min_v);
  // cv::Scalar upper = cv::Scalar((int)set_conf.max_y, (int)set_conf.max_u,  (int)set_conf.max_v);
  

    cv::Scalar lower = cv::Scalar(56, 69, 64);
    cv::Scalar upper = cv::Scalar(123,160,157);

  // cv::Scalar lower = cv::Scalar(40, 65, 160);
  // cv::Scalar upper = cv::Scalar(145,140,225);


  // cv::Scalar lower = cv::Scalar(41, 53, 134);
  // cv::Scalar upper = cv::Scalar(183,121,249);


  // cv::Scalar lower = cv::Scalar(75, 117, 122);
  // cv::Scalar upper = cv::Scalar(115, 151, 144);

  // cv::Scalar lower = cv::Scalar(0 ,120, 120);
  // cv::Scalar upper = cv::Scalar(171,140, 140);

  cv::Rect crop(0, 140, 240, 240);
  cropped = imageyuv(crop);
  cv::resize(cropped,cropped, cv::Size(60, 60), 0, 0);

  cv::Mat M = cv::getRotationMatrix2D(cv::Point(30,30), 90.0, 1);
  cv::warpAffine(cropped, cropped, M, cv::Point(60,60));

  // cv::Scalar lower = cv::Scalar(128 ,78, 0);
  // cv::Scalar upper = cv::Scalar(133,111, 255);

  cv::inRange(cropped, lower, upper, mask);
  // cv::bitwise_not(mask, mask);

  vector<cv::Point2i> positions;
  // grayscale_opencv_to_yuv422(mask, out, width, height);

  cv::Canny(mask, edges, 300, 400);

  // grayscale_opencv_to_yuv422(mask, out, width, height);
  cv::Size edges_size = edges.size();


  vector<cv::Point2i> points;

  bool found_points = false;
  for(int x = 0; x < 60; x+=2)
  {
    for(int y = 60; y > 0; y--)
    {
      if(edges.at<uint8_t>(y, x) == 0)
      {
        int sum = 0;
        for(int k = 0; k<4; k++)
        {
          sum += mask.at<uint8_t>(y-k,x);
        }
        if(sum == 0)
        {
          points.push_back(cv::Point2i(x,y));
          found_points = true;
          break;
        }
      }
    }
  }


  coloryuv_opencv_to_yuv422(cropped, out, 60,60);


  std::vector<float> obs;

  float FOV_y = 80.0f/180.0f*3.14159f;
  float alt = 1;

  float n_obs = 0;
  float n_obs_left = 0;
  float n_obs_right = 0;

  for(int i = 0; i<points.size(); i++)
  {
    float theta = (points[i].y*4 - 120)*(FOV_y/240);
    float d = alt/(tan(theta));
    cout << "theta:" << theta << " distance: " << d << " pixel: " << (points[i].y*4 - 120)  << endl;
    obs.push_back(d);
    cv::line(cropped, cv::Point(points[i].x, points[i].y), cv::Point(points[i].x, points[i].y-4), cv::Scalar(int(d*20))); 

    if (d < threshold)
    {
      n_obs++;
      if(12 > points[i].x)
      {
        n_obs_left++;
      }
      else if(48 < points[i].x)
      {
        n_obs_right++;
      }

    }
  }

  cout << "obs left: " << n_obs_left << endl;
  cout << "obs right: " << n_obs_right << endl;

  int control;
  if(n_obs_left > obs_threshold && n_obs_left > n_obs_right)
  {
    control = 1;
    cout << "obs left, going right" << endl;  
  }
  else if(n_obs_right > obs_threshold && n_obs_left < n_obs_right)
  {
    control = -1;
    cout << "obs right, going left" << endl;  

  }
  else if(n_obs > total_threshold && n_obs_left > n_obs_right)
  {
    control = 1;
    cout << "obs ahead, going right" << endl;  

  }
  else if(n_obs > total_threshold && n_obs_left < n_obs_right)
  {
    control = -1;
    cout << "obs ahead, going left" << endl;  

  } 
  else
  {
    control = 0;
    cout << "going straight" << endl;  

  }


  coloryuv_opencv_to_yuv422(cropped, out, 60,60);


  return control;
}
