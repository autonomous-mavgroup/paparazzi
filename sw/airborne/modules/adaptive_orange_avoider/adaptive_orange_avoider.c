/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/adaptive_orange_avoider/adaptive_orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);
bool close_to_waypoint(uint8_t);
void print_current_state(void);
void print_waypoint_pos(uint8_t);
void test_list_points(void);
void test_find_distances(void);
struct EnuCoor_f wpToPoint(uint8_t);

enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    color_count = quality;
}
uint8_t corner_wps[4] = {WP__OZ1, WP__OZ2, WP__OZ3, WP__OZ4};
uint8_t cyberzoo_corners[4] = {WP__CZ1, WP__CZ2, WP__CZ3, WP__CZ4};
int current_target = 0;

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void adaptive_orange_avoider_init(void)
{
    // Initialise random values
    srand(time(NULL));
    chooseRandomIncrementAvoidance();

    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void adaptive_orange_avoider_periodic(void)
{
    // only evaluate our state machine if we are flying
    if(!autopilot_in_flight()){
        return;
    }
    // compute current color thresholds
    int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

    VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count, navigation_state);

    // update our safe confidence using color threshold
    if(color_count < color_count_threshold){
        obstacle_free_confidence++;
    } else {
        obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
    }

    // bound obstacle_free_confidence
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

    float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);
    PRINT("Navigation state %d \n", navigation_state);
    print_current_state();
    for (int i = 0; i < 4; i++){
        print_waypoint_pos(cyberzoo_corners[i]);
    }
    switch (navigation_state){
        case SAFE:

            if (close_to_waypoint(WP_GOAL))
            {
                PRINT("Waypoint close, moving target \n");
                current_target = (current_target + 1) % 3;
                struct EnuCoor_i new_coor;
                new_coor.x = waypoints[corner_wps[current_target]].enu_i.x;
                new_coor.y = waypoints[corner_wps[current_target]].enu_i.y;
                moveWaypoint(WP_GOAL, &new_coor);
            }

            moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
            if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                navigation_state = OUT_OF_BOUNDS;
            } else if (obstacle_free_confidence == 0){
                navigation_state = OBSTACLE_FOUND;
            } else {
//        moveWaypointForward(WP_GOAL, moveDistance);
            }

            break;
        case OBSTACLE_FOUND:
            // stop
            waypoint_set_here_2d(WP_GOAL);
            waypoint_set_here_2d(WP_TRAJECTORY);

            // randomly select new search direction
            chooseRandomIncrementAvoidance();

            navigation_state = SEARCH_FOR_SAFE_HEADING;

            break;
        case SEARCH_FOR_SAFE_HEADING:
            increase_nav_heading(heading_increment);

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                navigation_state = SAFE;
            }
            break;
        case OUT_OF_BOUNDS:
            increase_nav_heading(heading_increment);
            moveWaypointForward(WP_TRAJECTORY, 1.5f);

            if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                // add offset to head back into arena
                increase_nav_heading(heading_increment);

                // reset safe counter
                obstacle_free_confidence = 0;

                // ensure direction is safe before continuing
                navigation_state = SEARCH_FOR_SAFE_HEADING;
            }
            break;
        default:
            break;
    }
    return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
    float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

    // normalize heading to [-pi, pi]
    FLOAT_ANGLE_NORMALIZE(new_heading);

    // set heading
    nav_heading = ANGLE_BFP_OF_REAL(new_heading);

    VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
    return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
    float heading  = stateGetNedToBodyEulers_f()->psi;

    // Now determine where to place the waypoint you want to go to
    new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
    new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
    VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                  POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                  stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
    return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
    VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                  POS_FLOAT_OF_BFP(new_coor->y));
    waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
    return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
    struct EnuCoor_i new_coor;
    calculateForwards(&new_coor, distanceMeters);
    moveWaypoint(waypoint, &new_coor);
    return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
    // Randomly choose CW or CCW avoiding direction
    if (rand() % 2 == 0) {
        heading_increment = 5.f;
        VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
    } else {
        heading_increment = -5.f;
        VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
    }
    return false;
}

float square_f(float x){
    return x * x;
}


bool close_to_waypoint(uint8_t wp_id){
    float distance = sqrtf(square_f(waypoints[wp_id].enu_f.x - stateGetPositionEnu_f()->x) +
                           square_f((waypoints[wp_id].enu_f.y - stateGetPositionEnu_f()->y)));
    return distance < 0.3;
}

void print_current_state(void){
    float x = stateGetPositionEnu_f()->x;
    float y = stateGetPositionEnu_f()->y;
    float wp_x = waypoint_get_x(WP_TRAJECTORY);
    float wp_y = waypoint_get_y(WP_TRAJECTORY);
    PRINT("Current position (%f, %f) \n WP_TRAJ (%f, %f) \n", x, y, wp_x, wp_y);
}

void print_waypoint_pos(uint8_t wp){
    PRINT("Wp position (%f, %f) \n ", waypoint_get_x(wp), waypoint_get_y(wp));
}

void printPointPos(struct EnuCoor_f* point) {
    PRINT("Point Position: (%f, %f) \n", point->x, point->y);
}

typedef struct ListPoints ListPoints;

float distancePointsf(struct EnuCoor_f* start_point, struct EnuCoor_f* end_point){
    float distance = sqrtf(powf(end_point->x - start_point->x, 2) + powf(end_point->y - start_point->y,2));
    return distance;
}

struct ListPoints{
    float x[20];
    float y[20];
    int number_points;
};

ListPoints generateIntermidiatePoints(struct EnuCoor_f* start_point, struct EnuCoor_f* end_point, int points_num){
    float v_x = end_point->x - start_point->x;
    float v_y = end_point->y - start_point->y;
    ListPoints list_of_points;
    list_of_points.number_points = points_num;
    for (int i = 0; i < points_num + 1; i++){
        float x_i = start_point->x + v_x/points_num * i;
        float y_i = start_point->y + v_y/points_num * i;
        list_of_points.x[i] = x_i;
        list_of_points.y[i] = y_i;
    }
    return list_of_points;
}

ListPoints generetatePossibleTargets(uint8_t corner_id, uint8_t* corner_wps, int points_num){
    struct EnuCoor_f corner_point = wpToPoint(corner_wps[corner_id]);
    struct EnuCoor_f corner_point_next =  wpToPoint(corner_wps[(corner_id + 1) % 4]);
    ListPoints line = generateIntermidiatePoints(&corner_point, &corner_point_next, points_num);
    return line;
}

float* findDistances(ListPoints* points){
    float distances[points->number_points];
    struct EnuCoor_f current_position = {.x = stateGetPositionEnu_f()->x, .y = stateGetPositionEnu_f()->y};
    for (int i = 0; i < points->number_points + 1; i++){
        struct EnuCoor_f point = {.x = points->x[i], .y = points->y[i]};
        distances[i] = distancePointsf(&current_position, &point);
    }
    return distances;
}

struct EnuCoor_f wpToPoint(uint8_t wp_id){
    struct EnuCoor_f point;
    point.x = waypoint_get_x(wp_id);
    point.y = waypoint_get_y(wp_id);
    return point;
}

struct EnuCoor_i float2IntPoint(struct EnuCoor_f* point){
    struct EnuCoor_i point_int;
    point_int.x = POS_BFP_OF_REAL(point->x);
    point_int.y = POS_BFP_OF_REAL(point->y);
    return point_int;
}

void test_list_points(void) {
    struct EnuCoor_f point_1 = {.x = 0, .y = 0};
    struct EnuCoor_f point_2 = {.x = 0, .y = 1};
    ListPoints list_of_points = generateIntermidiatePoints(&point_1, &point_2, 10);
    for (int i = 0; i < 10 + 1; i++) {
        PRINT("(x, y) = (%f, %f)", list_of_points.x[i], list_of_points.y[i]);
    }
}

void test_find_distances(void){
//    struct EnuCoor_f current_position = {.x = waypoint_get_x(corner_wps[0]), .y = waypoint_get_y(corner_wps[1])};
//    stateSetPositionEnu_f(&current_position);
    ListPoints list_of_points = generetatePossibleTargets(2, corner_wps, 10);
    float* distances = findDistances(&list_of_points);
    for (int i = 0; i < list_of_points.number_points + 1; i ++){
        PRINT("Point (%f, %f), distance : %f", list_of_points.x[i], list_of_points.y[i], distances[i]);
    }
}