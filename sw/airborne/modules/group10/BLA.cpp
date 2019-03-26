//
// Created by matteo on 3/25/19.
//

#include "BLA.h"

#include "navigation.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <array>
#include <cmath>

// ------------TUNABLE PARAMETERS-------------
#define BLACK_THRESH 20  // filt_black
#define GRID_Y 30        // find_white
#define GRID_X 54        // find_white
#define LINE_SKIP 1      // find_safe_vertical
#define D_MARGIN 1.5     // edge_reached_check
#define HALF_PI 1.57080


void filt_black(const cv::Mat &img, cv::Mat &filt_img, const int &black_thresh)
{
    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++)
        {
            if (img[i, j, 0] < black_thresh)
                filt_img[i, j] = 0;
        }
    }
}


cv::Mat find_white(const cv::Mat &filt_img, const std::array<int, 2> &grid_size,
        const int &height, const int &width)
{
    int y_step = height / grid_size[0];
    int x_step = width / grid_size[1];

    cv::Mat ret = Mat::zeros(grid_size[0], grid_size[1], CV_16U)
    for (int i = 0; i < grid_size[0]; i++){
        for (int j = 0; j < grid_size[1]; j++)
        {
            ret[i, j] = cv::sum(filt_img(cv::Range(i*y_step, (i+1)*y_step),
                    cv::Range(j*x_step, (j+1)*x_step)));
        }
    }

    return ret;
}


std::array<int, 2> find_safe_vertical(const cv::Mat &sum_arr, const int &line_skip,
        const int &height, const int &width)
{
    int seq_len = 0;
    int seq_start = 0;
    int seq_end = 0;
    int seq_line = 0;

    int counter;
    int start;
    for (int line = 0; line < height; line += line_skip)
    {
        counter = 0;
        start = 0;

        for (int idx = 0; idx < width; idx++)
        {
            if (counter == 0 && sum_arr[line, idx] == 0) {
                start = idx;
                counter += 1;
            } else if (sum_arr[line, idx] == 0) {
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
    int angle_from_center = 0.5*FOV_x * (px_from_center / (grid_size[1]/2));

    return angle_from_center;
}


bool edge_reached_check(const cv::Mat &sum_arr, const int &grid_y, const std::array<int, 2> &safe_point,
        const float &drone_height, const float &drone_pitch, const int &max_sum, const float &FOV_y,
        const double &d_margin)
{
    float d_img_bot = drone_height * tan(HALF_PI - 0.5 * FOV_y + drone_pitch);
    float theta = atan2(drone_height, d_img_bot + d_margin);
    float theta_px = 0.5 * grid_y * (theta / (0.5*FOV_y));

    white_idx = 0;
    for (int idx = safe_point[0]; idx < grid_y; idx++)
    {
        if (sum_arr[idx, safe_point[1]] == max_sum)
        {
            white_idx = idx;
            break;
        }
    }

    bool edge_reached = false;
    if (white_idx > theta_px) {edge_reached = true;}

    return edge_reached;
}


BLA_ret BLA(char *img, int height, int width)
{
    // Drone state variables
    EnuCoor_f* drone_state = stateGetPositionEnu_f();
    float drone_height = drone_state->z;

    struct FloatEulers* drone_attitude = stateGetNedToBodyEulers_f();

    float FOV_x = 120.0f/180.0f*3.14159f;
    float FOV_y = 120.0f/180.0f*3.14159f;

    // Load frame
    cv::Mat M(height, width, CV_8UC2, img);
    cv::rotate(M, M, cv::ROTATE_90_COUNTERCLOCKWISE);

    cv::Mat filt_img = cv::Mat::ones(height, width, CV_8U);
    filt_black(M, filt_img, BLACK_THRESH);

    std::array<int, 2> grid_size {GRID_Y, GRID_X};
    cv::Mat sum_arr = find_white(filt_img, grid_size, height, width);

    std::array<int, 2> safe_point = find_safe_vertical(sum_arr, LINE_SKIP, height, width);

    int heading;
    bool edge_reached;
    if (safe_point[0] != -1)
    {
        heading = new_heading(grid_size, safe_point[1], FOV_x);

        int max_sum = GRID_Y*GRID_X;
        edge_reached = edge_reached_check(sum_arr, GRID_Y, safe_point, drone_height, drone_attitude->theta,
                                               max_sum, FOV_y, D_MARGIN);
    }
    else
    {
        heading = 420;
        edge_reached = false;
    }


    // Initialize return argument
    struct BLA_ret ret = {.heading = heading, .edge_reached = edge_reached};

    return ret;
}