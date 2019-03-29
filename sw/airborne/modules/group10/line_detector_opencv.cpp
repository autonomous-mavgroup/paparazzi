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

int scale = 4;

int resize_width = 520;
int resize_height = 60;

int margin = resize_height>120 ? 0 : 120 - resize_height;

int scaled_width = resize_width / scale;
int scaled_height = resize_height / scale;

int scaled_width_center = scaled_width / 2;
int scaled_height_center = scaled_height / 2;

int resize_offset_width = (520 - resize_width)/2;
int resize_offset_height = 240 - resize_height;



float threshold = 1.4;
float obs_threshold = 10;
float total_threshold = 30;
float central_threshold = 6; // 7


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
  cv::Mat rotated;
  cv::Mat scaled;
  cv::Mat edges;
  cv::Mat res(height, width, CV_8UC2);
  // Image already in YUV format

  uyvy_opencv_to_yuv_opencv(imageyuv, image, 240, 520);;

   cv::Scalar lower = cv::Scalar(50, 120, 120);
   cv::Scalar upper = cv::Scalar(200,130,130);

  // cv::Scalar lower = cv::Scalar(56, 69, 64);
  // cv::Scalar upper = cv::Scalar(123,160,157);

  cv::Rect crop(0, resize_offset_width, resize_height, resize_width);
  cropped = imageyuv(crop);
  
  cv::rotate(cropped, cropped, cv::ROTATE_90_COUNTERCLOCKWISE);


  // scaled = rotated.clone();
  // cv::resize(scaled,scaled, cv::Size(), 0.25, 0.25);
  // coloryuv_opencv_to_yuv422(cropped, out, 520, 120);

  cv::inRange(cropped, lower, upper, mask);
  // cout << "scaled width: " << scaled_width_center << " scaled height: " << scaled_height_center << endl;

  // grayscale_opencv_to_yuv422(mask, out, width, height);


  vector<cv::Point2i> points;

  for(int x = 0; x < 520; x+=10)
  {
    for(int y = 60; y > 0; y--)
    {
      if(mask.at<uint8_t>(y, x) == 0)
      {
        int sum = 0;
        for(int k = 0; k<8; k++)
        {
          sum += mask.at<uint8_t>(y-k,x);
        }
        if(sum == 0)
        {
          points.push_back(cv::Point2i(x,y));
          break;
        }
      }
    }
  }


  // coloryuv_opencv_to_yuv422(cropped, out, 60,60);


  std::vector<float> obs;

  float FOV_y = 80.0f/180.0f*3.14159f;
  float alt = 1;

  float n_obs = 0;
  float n_obs_left = 0;
  float n_obs_right = 0;

  for(int i = 0; i<points.size(); i++)
  {
    float theta = ( 60 + points[i].y)*(FOV_y/240);
    float d = alt/(tan(theta));
    cout << "theta: " << theta << " distance: " << d << endl;
    obs.push_back(d);
    cv::line(cropped, cv::Point(points[i].x, points[i].y), cv::Point(points[i].x, points[i].y-16), cv::Scalar(255)); 

    if (d < threshold)
    {
      n_obs++;
      if(104 > points[i].x)
      {
        n_obs_left++;
      }
      else if(416 < points[i].x)
      {
        n_obs_right++;
      }

    }
  }

  cout << "obs left: " << n_obs_left << endl;
  cout << "obs: " << n_obs << endl;
  cout << "obs right: " << n_obs_right << endl;

  int control;
  if(n_obs > total_threshold)
  {
    control = 2;
    cout << "obs ahead, turning around" << endl;
  }
  else if(n_obs_right > obs_threshold && n_obs_left < n_obs_right)
  {
    control = -1;
    cout << "obs right, going left" << endl;  

  }

  //  else if(n_obs > total_threshold && n_obs_left > n_obs_right)
  //  {
  //    control = 1;
  //    cout << "obs ahead, going right" << endl;
  //
  //  }
  //  else if(n_obs > total_threshold && n_obs_left < n_obs_right)
  //  {
  //    control = -1;
  //    cout << "obs ahead, going left" << endl;
  //
  //  }
  else if(n_obs_left > obs_threshold && n_obs_left > n_obs_right)
  {
    control = 1;
  cout << "obs left, going right" << endl;

  }
  else if((n_obs - n_obs_right - n_obs_left)> central_threshold && n_obs_left < n_obs_right)
  {
  control = -1;
  cout << "obs in front, going left" << endl;

  }
  else if((n_obs - n_obs_right - n_obs_left)> central_threshold && n_obs_left > n_obs_right)
  {
    control = 1;
    cout << "obs in front, going right" << endl;

  }
  else if((n_obs - n_obs_right - n_obs_left) > central_threshold && n_obs_left == n_obs_right)
    {
      control = 1;
      cout << "obs in front, going right" << endl;

    }
  else
  {
    control = 0;
    cout << "going straight" << endl;  

  }

  coloryuv_opencv_to_yuv422(cropped, out, 520, 60);



  return control;
}
