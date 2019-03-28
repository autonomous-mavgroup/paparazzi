//
// Created by matteo on 3/25/19.
//

#include "BLA.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <array>
#include <cmath>
#include <stdio.h>
#include <string.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// ------------TUNABLE PARAMETERS-------------
#define BLACK_THRESH 70  // filt_black
#define GRID_Y 55        // find_white
#define GRID_X 30        // find_white
#define LINE_SKIP 1      // find_safe_vertical
#define D_MARGIN 1.5     // edge_reached_check
#define HALF_PI 1.57080

//#define DEBUG 1

void grayscale_opencv_to_yuv422(cv::Mat image, char *img);

cv::Mat filt_black(const cv::Mat &img, const int height, const int width,
                const int &black_thresh);

cv::Mat find_white(const cv::Mat &filt_img, const std::array<int, 2> &grid_size, const int &height,
                   const int &width);

std::array<int, 2> find_safe_vertical(const cv::Mat &sum_arr, const int line_skip,
                                      const int height, const int width);

int new_heading(const std::array<int, 2> &grid_size, const int &vertical, const float &FOV_x);

bool edge_reached_check(const cv::Mat &sum_arr, const int &grid_y, const std::array<int, 2> &safe_point,
                        const float &drone_height, const float &drone_pitch, const int &max_sum,
                        const float &FOV_y, const double &d_margin);


void grayscale_opencv_to_yuv422(cv::Mat image, char *img)
{
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


cv::Mat filt_black(const cv::Mat &img, const int height, const int width,
        const int &black_thresh)
{
    cv::Mat filt_img = cv::Mat::ones(height, width, CV_8UC1);
    for (int i = 0; i < height; i++){
        const uchar* img_ptr = img.ptr(i);
        for (int j = 0; j < width; j++)
        {
            const uchar* px = img_ptr;
            if (px[1] < black_thresh)
            {
                filt_img.at<uint8_t>(i, j) = 0;
            }
            // DEBUG
//            else {
//                filt_img.at<uint8_t>(i, j) = 255;}
            img_ptr += 2;
        }
    }
    return filt_img;
}


cv::Mat find_white(const cv::Mat &filt_img, const std::array<int, 2> &grid_size,
        const int &height, const int &width)
{
    int y_step = height / grid_size[0];
    int x_step = width / grid_size[1];
    double max_sum = y_step * x_step;

    cv::Mat ret;

    ret = cv::Mat::zeros(grid_size[0], grid_size[1], CV_8UC1);
    int sum;
    for (int i = 0; i < ret.rows; i++)
    {
        for (int j = 0; j < ret.cols; j++)
        {
            cv::Mat tmp_mat = filt_img(cv::Range(i*y_step, (i+1)*y_step),
                                       cv::Range(j*x_step, (j+1)*x_step));
            sum = 0;
            for (int sub_row = 0; sub_row < tmp_mat.rows; sub_row++)
            {
                const uchar* tmp_ptr = tmp_mat.ptr(sub_row);
                for (int sub_col = 0; sub_col < tmp_mat.cols; sub_col++)
                {
                    const uchar* elm = tmp_ptr;
                    sum += elm[0];
                    tmp_ptr += 1;
                }
            }

            ret.at<uint8_t>(i, j) = sum; // / max_sum * 255;
        }
    }
    return ret;
}


std::array<int, 2> find_safe_vertical(const cv::Mat &sum_arr, const int line_skip,
        const int height, const int width)
{
    int seq_len = 0;
    int seq_start = 0;
    int seq_end = 0;
    int seq_line = 0;

    int counter;
    int start;
    int idx;
    for (int line = 0; line < height; line++)
    {
        counter = 0;
        start = 0;
        for (idx = 0; idx < width; idx++)
        {
            if ((counter == 0) && (sum_arr.at<uint8_t>(line, idx) <= 1)) {
                start = idx;
                counter += 1;
            } else if (sum_arr.at<uint8_t>(line, idx) <= 1) {
                counter += 1;
            } else {
                if (counter > seq_len) {
                    seq_start = start;
                    seq_end = idx - 1;
                    seq_len = counter;
                    seq_line = line;
                    counter = 0;
                } else {counter = 0;}
            }
        }
        if (counter > seq_len) {
            seq_start = start;
            seq_end = idx - 1;
            seq_len = counter;
            seq_line = line;
            counter = 0;
        }
    }
    std::array<int, 2> ret {seq_line, (seq_start + seq_end) / 2};
    if (seq_len < 4)
    {
        ret[0] = -1;
        ret[1] = -1;
    }
    return ret;
}


int new_heading(const std::array<int, 2> &grid_size, const int &vertical, const float &FOV_x)
{
    int px_from_center = vertical - grid_size[1] / 2;
    int angle_from_center = (180. / M_PI) * 0.5*FOV_x * (px_from_center / (grid_size[1]*0.5));
    return angle_from_center;
}


bool edge_reached_check(const cv::Mat &sum_arr, const int &grid_y, const std::array<int, 2> &safe_point,
        const float &drone_height, const float &drone_pitch, const int &max_sum, const float &FOV_y,
        const double &d_margin)
{
    float d_img_bot = drone_height * tan(HALF_PI - 0.5 * FOV_y + drone_pitch);
    float theta = atan2(drone_height, d_img_bot + d_margin);
    float theta_px = 0.5 * grid_y * (theta / (0.5*FOV_y));

    int white_idx = 0;
    for (int idx = safe_point[0]; idx < grid_y; idx++)
    {
        if (sum_arr.at<uint8_t>(idx, safe_point[1]) == max_sum)
        {
            white_idx = idx;
            break;
        }
    }

    bool edge_reached = false;
    if (white_idx > theta_px) {edge_reached = true;}

    return edge_reached;
}


BLA_ret BLA(char *img, int height, int width, float drone_height, float drone_theta, char *out)
{
    float FOV_x = 120.0f/180.0f*3.14159f;
    float FOV_y = 80.0f/180.0f*3.14159f;

    // Load frame
    cv::Mat M(height, width, CV_8UC2, img);

    cv::Mat filt_img = filt_black(M, height, width, BLACK_THRESH);

    std::array<int, 2> grid_size {GRID_Y, GRID_X};

    cv::Mat sum_arr = find_white(filt_img, grid_size, height, width);

    grayscale_opencv_to_yuv422(sum_arr, out);

    std::array<int, 2> safe_point = find_safe_vertical(sum_arr, LINE_SKIP, grid_size[0], grid_size[1]);

    int heading;
    bool edge_reached;
    if (safe_point[0] != -1)
    {
        heading = new_heading(grid_size, safe_point[1], FOV_x);
        int y_step = height / grid_size[0];
        int x_step = width / grid_size[1];
        int max_sum = y_step * x_step;
        edge_reached = edge_reached_check(sum_arr, GRID_Y, safe_point, drone_height, drone_theta,
                                               max_sum, FOV_y, D_MARGIN);
        PRINT("\n\nheading: %d, %d\n\n", heading, edge_reached);
    }
    else
    {
        heading = 420;
        edge_reached = false;
    }

    struct BLA_ret ret = {.heading = heading, .edge_reached = edge_reached};

    return ret;
}