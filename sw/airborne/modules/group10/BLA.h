//
// Created by matteo on 3/25/19.
//

#ifndef PAPARAZZI_BLA_H
#define PAPARAZZI_BLA_H

#ifdef  __cplusplus
extern "C" {
#endif

struct BLA_ret
{
    int heading;
    bool edge_reached;
};

void filt_black(const cv::Mat &img, cv::Mat &filt_img, const int &black_thresh);

cv::Mat find_white(const cv::Mat &filt_img, const std::array<int, 2> &grid_size, const int &height,
                   const int &width);

std::array<int, 2> find_safe_vertical(const cv::Mat &sum_arr, const int &line_skip,
                                      const int &height, const int &width);

float new_heading(const std::array<int, 2> &grid_size, const int &vertical, const float &FOV_x);

bool edge_reached_check(const cv::Mat &sum_arr, const int &grid_y, const std::array<int, 2> &safe_point,
                        const float &drone_height, const float &drone_pitch, const int &max_sum,
                        const float &FOV_y, const double &d_margin);

BLA_ret BLA(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif