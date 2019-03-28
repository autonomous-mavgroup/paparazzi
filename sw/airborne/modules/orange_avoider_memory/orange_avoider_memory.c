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
 * @file "modules/orange_avoider_memory/orange_avoider_memory.c"
 * @author Lorenzo Terenzi
 * 
 */

#include "modules/orange_avoider_memory/orange_avoider_memory.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include "modules/orange_avoider_memory/graph.h"
#include "modules/orange_avoider_memory/shortest_path.h"
#include <stdio.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"
#define DIJKSTRA_DEBUG
#define ORANGE_AVOIDER_VERBOSE TRUE

#if defined(DIJKSTRA_DEBUG)
#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#else
#define PRINT(string, ...)
#endif

#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif
uint8_t increase_nav_heading(float);
int rotate(int);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t chooseRandomIncrementAvoidance(int);
// States for Finite State Machine
enum navigation_state_t {
    safe_state,
    obstacle_found_state,
    search_heading_state,
    out_of_bounds_state,
    rerouting_state,
    validate_node_state
};

// define settings
float oa_color_count_frac = 0.18f;
bool search_clockwise = 0;
float heading_change = 0;
// define and initialise global variables
enum navigation_state_t navigation_state = safe_state;
int32_t orange_count = 0;// orange color count from color filter for obstacle detection
int32_t green_count = 0;
int orange_x = 0;
int orange_y = 0;
int32_t black_count = 0;
int16_t obstacle_free_confidence = 5;   // a meagsure of how certain we are that the way ahead is safe.
float heading_increment = 90.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID

#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event orange_detection_ev;
static void orange_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t pixel_x, int16_t pixel_y,
                                int16_t __attribute__((unused)) pixel_width,
                                int16_t __attribute__((unused)) pixel_height,
                                int32_t quality, int16_t __attribute__((unused)) extra)
{
    orange_x = pixel_x;
    orange_y = pixel_y;
    orange_count = quality;
}
static abi_event green_detection_ev;
static void green_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    green_count = quality;
}
static abi_event black_detection_ev;
static void black_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    black_count = quality;
}
//uint8_t corner_wps[4] = {WP_0, WP_1, WP_2, WP_3};
uint8_t corner_wps[2] = {WP_1, WP_3};
uint8_t cyberzoo_corners[4] = {WP__CZ1, WP__CZ2, WP__CZ3, WP__CZ4};


void print_waypoint_pos(uint8_t wp){
    PRINT("Wp %i position (%f, %f) \n ", wp, waypoint_get_x(wp), waypoint_get_y(wp));
}

void print_point_pos(struct EnuCoor_i* point){
    PRINT("\n Point position : (%f, %f) ", POS_FLOAT_OF_BFP(point->x), POS_FLOAT_OF_BFP(point->y));
}

typedef struct Node{
    struct EnuCoor_i position;
    int name;
} Node;

void set_target(Node*);

typedef struct ListNodes{
    struct Node node;
    struct ListNodes* next;
} *ListNodes;

ListNodes init_list(){
    ListNodes list;
    list = (ListNodes)malloc(sizeof(struct ListNodes));
    list->next = NULL;
    return list;
}

Node* find_node(ListNodes head, int node_name){
    ListNodes p = head;
    while(p != NULL){
        if (p->node.name == node_name){
            return &p->node;
        }
        p = p->next;
    }
    return NULL;
}

ListNodes add_node(ListNodes head, Node node){
    Node* exist_node_ptr = find_node(head, node.name);
    if (exist_node_ptr == NULL){
        ListNodes new_nodes_ptr, p;
        new_nodes_ptr = init_list();
        new_nodes_ptr->node = node;
        if (head == NULL){
            head = new_nodes_ptr;
        }
        else{
            p = head;
            while(p->next != NULL){
                p = p->next;
            }
            p->next = new_nodes_ptr;
        }
    }
    return head;
}

typedef struct Route{
    graph_t* graph;
    ListNodes list_nodes;
    Path* current_path;
} Route;

int get_new_node_name(ListNodes head){
    int counter = 0;
    ListNodes p;
    p = head;
    while(p != NULL){
        p = p->next;
        counter = counter + 1;
    }
    return counter + 'a';
}


float distancePoints(struct EnuCoor_i* start_point, struct EnuCoor_i* end_point){
    float distance = sqrt(powf(end_point->x - start_point->x, 2) + powf(end_point->y - start_point->y,2));
    return distance;
}

void add_path(Route* route, Node point_1, Node point_2){
    // distance in centimeters
    route->list_nodes = add_node(route->list_nodes, point_1);
    route->list_nodes = add_node(route->list_nodes, point_2);
    int distance = distancePoints(&point_1.position, &point_2.position);
    add_double_edge(route->graph, point_1.name, point_2.name, distance);
}

void block_path(Route* route, Node* point_1, Node* point_2){
    block_edge(route->graph, point_1->name, point_2->name);
}

Path find_path(Route* route, Node* point_start, Node* point_end){
    dijkstra(route->graph, point_start->name, point_end->name);
    Path path = get_path(route->graph, point_end->name);
    return path;
}


void load_path(Route* route, Path* path){
    Node* target;
    route->current_path = path;
    path->target_node = 1;
    target = find_node(route->list_nodes, path->path[path->target_node]);
    set_target(target);
}

Node* next_in_route(Route *route){
    return find_node(route->list_nodes, route->current_path->path[route->current_path->target_node]);
}

void load_next_wp(Route* route, Node* target){
    route->current_path->target_node += 1;
    target = find_node(route->list_nodes, route->current_path->path[route->current_path->target_node]);
    set_target(target);
}

bool completed_path(Route* route){
    return route->current_path->target_node == route->current_path->number_nodes - 1;
}

struct EnuCoor_i get_wp_pos_enui(uint8_t wp_id){
    struct EnuCoor_i point;
    point.x = waypoints[wp_id].enu_i.x;
    point.y = waypoints[wp_id].enu_i.y;
    point.z = waypoints[wp_id].enu_i.z;
    return point;
}

bool close_to_node(Node *target) {
    return get_dist2_to_point(&target->position) < 0.5 * 0.5;
}

bool close_to_wp(uint8_t wp) {
    struct EnuCoor_i pos_wp = get_wp_pos_enui(wp);
    return get_dist2_to_point(&pos_wp) < 0.3 * 0.3;
}

int select_rand_target(int avoid_this_num, int number_wps){
    int num = avoid_this_num;
    while( num == avoid_this_num){
        num = (rand() % number_wps);
    }
    return num;
};

Node* get_target(Route* route){
    return find_node(route->list_nodes, route->current_path->target_node + 'a');
}

void set_target(Node* target){
    navigation_target = target->position;
    nav_set_heading_towards_target();
    waypoint_set_enu_i(WP_GOAL, &(target->position));
}

graph_t* g;
Route route;
Path current_path;
// not references to avoid segfaults when nodes are deallocated in the initialization
Node last_node;
Node target_node;
Node deviation_node;
bool first_run = 1;
int last_wp_route;
int next_wp_route;
int number_waypoints = 2;

void orange_avoider_init() {
    srand(time(NULL));
    // Initialize initial graph
    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &orange_detection_ev, orange_detection_cb);
    AbiBindMsgVISUAL_DETECTION(2, &green_detection_ev, green_detection_cb);
    AbiBindMsgVISUAL_DETECTION(3, &black_detection_ev, black_detection_cb);


}

void graph_init_corners(){
    g = calloc(1, sizeof (graph_t));
    waypoints_init();
    route.graph = g;
    PRINT("FUNC X : %f \n", waypoint_get_x(WP_0));
    struct EnuCoor_i pos_a = get_wp_pos_enui(WP_0);
    PRINT("Position x y : (%i, %i)", pos_a.x, pos_a.y);
    Node a = {.name = 'a', .position = pos_a};
    struct EnuCoor_i pos_b = get_wp_pos_enui(WP_1);
    Node b = {.name = 'b', .position = pos_b};
    struct EnuCoor_i pos_c = get_wp_pos_enui(WP_2);
    Node c = {.name = 'c', .position = pos_c};
    struct EnuCoor_i pos_d = get_wp_pos_enui(WP_3);
    Node d = {.name = 'd', .position = pos_d};
    // Init Route
    // add connectivity
    add_path(&route, a, c);
    add_path(&route, a, d);
    add_path(&route, b, c);
    add_path(&route, b, d);
    // set initial position
    Node home = {.name = 'e', .position = *stateGetPositionEnu_i()};
    last_node = home;
    target_node = *find_node(route.list_nodes, 1 + 'a');
    next_wp_route = 1;
    // init value
    add_path(&route, last_node, target_node);
    current_path = find_path(&route, &last_node, &target_node);
}

void graph_init_test(){
    g = calloc(1, sizeof (graph_t));
    waypoints_init();
    route.graph = g;
    struct EnuCoor_i pos_a = get_wp_pos_enui(WP_0);
    Node a = {.name = 'a', .position = pos_a};
    struct EnuCoor_i pos_b = get_wp_pos_enui(WP_2);
    Node b = {.name = 'b', .position = pos_b};

    // Init Route
    // add connectivity
    add_path(&route, a, b);

    // set initial position
    Node home = {.name = 'c', .position = *stateGetPositionEnu_i()};
    last_node = home;
    target_node = *find_node(route.list_nodes, 'b');
    next_wp_route = 1;

    add_path(&route, last_node, target_node);
    current_path = find_path(&route, &last_node, &target_node);

}

int check_obstacle_presence(){
    int pixels = front_camera.output_size.w * front_camera.output_size.h;
    int green_min_treshold = 0 * pixels; //0.2
    int green_intermediate_treshold = 0.f * pixels; //0/3
    int black_max_treshold = 1 * pixels; //0.7
    int orange_intermediate_treshold = 0.18f * pixels; //0/15
    VERBOSE_PRINT("CONFIDENCE LEVEL :%i \n",obstacle_free_confidence);
    if (green_count < green_min_treshold){
        obstacle_free_confidence -= 1;
        VERBOSE_PRINT("GREEN FAIL: %f \n",(100.*green_count)/pixels);
        return 2;
    } else if(green_count > green_min_treshold) {
      if (orange_count > orange_intermediate_treshold && (orange_y < 100 && orange_y > -100)) {
        if (orange_count > orange_max_treshold){
          obstacle_free_confidence -= 3;
        } else{
        obstacle_free_confidence -= 1;
        }
        VERBOSE_PRINT("Orange FAIL: %f \n", (100. * orange_count) / pixels);
        return 1;
      } else if (black_count > black_max_treshold && green_count < green_intermediate_treshold) {
        obstacle_free_confidence -= 2;
        VERBOSE_PRINT("BLACK FAIL: %f \n", (100. * black_count) / pixels);
        return 3;
      } else {
        obstacle_free_confidence += 2;
        return 0;
      }
    }
}


void orange_avoider_periodic() {

    // printf causes segmentation fautls apparently
    // only evaluate our state machine if we are flying
    int failure_case = 0;
    if(!autopilot_in_flight()){
        return;
    }
    if (first_run){
        graph_init_test();
        load_path(&route, &current_path);
        target_node = *next_in_route(&route);
        first_run = 0;
    }
    // compute current color thresholds
    int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
    VERBOSE_PRINT("Current state %d \n", navigation_state);
    // update our safe confidence using color threshold
    if (!close_to_wp(corner_wps[next_wp_route])) {
        failure_case = check_obstacle_presence();
    }
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

    float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

    // FMS implementation
    switch (navigation_state){
        case safe_state: //0
            heading_change = 0;
            moveWaypointForward(WP_TRAJECTORY, 1.f * moveDistance);
            if (obstacle_free_confidence < 2 && !close_to_wp(next_wp_route)){
                    navigation_state = obstacle_found_state;
            }

            if (close_to_node(&target_node)){
                last_node = target_node;
                if (completed_path(&route)){
                    last_wp_route = next_wp_route;
                    next_wp_route = select_rand_target(last_wp_route, number_waypoints);
                    Node* next_wp = find_node(route.list_nodes, next_wp_route + 'a');
                    current_path = find_path(&route, &last_node, next_wp);
                    load_path(&route, &current_path);
                    target_node = *next_in_route(&route);
                } else {
                    // update the goal waypoint
                    last_node = target_node;
                    load_next_wp(&route, &target_node);
                    target_node = *next_in_route(&route);
                }
            }
            break;

        case validate_node_state: //5
            if (obstacle_free_confidence  <  2) {
                navigation_state = obstacle_found_state;
            } else if (obstacle_free_confidence > 3 && close_to_wp(WP_GOAL)) {
                deviation_node.name = get_new_node_name(route.list_nodes);
                deviation_node.position = get_wp_pos_enui(WP_GOAL);
                add_path(&route, last_node, deviation_node);
                add_path(&route, deviation_node, target_node);
                block_edge(route.graph, last_node.name, target_node.name);

                last_node = deviation_node;
                set_target(&target_node);
                navigation_state = safe_state;
            }
            else if (close_to_wp(WP_GOAL)){
                PRINT("REROUTING");
                navigation_state = rerouting_state;
            }
            // comment above else if statemennt and
            // uncomment following line to make the drone go in straight line until it sees as obstacle
//            else{
//                navigation_state = rerouting_state;
//            }
            break;
        case rerouting_state: //4
            moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
            if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                navigation_state = out_of_bounds_state;
            } else if (obstacle_free_confidence  <  2){
                navigation_state = obstacle_found_state;
            } else {
                moveWaypointForward(WP_GOAL, 1.2f *moveDistance);
                navigation_state = validate_node_state;
            }
            break;

        case obstacle_found_state: //1

            waypoint_set_here_2d(WP_GOAL);
            waypoint_set_here_2d(WP_TRAJECTORY);

            // randomly select new search direction
            chooseRandomIncrementAvoidance(failure_case);
            navigation_state = search_heading_state;
            break;

        case search_heading_state: //2
            if (obstacle_free_confidence >= 2){
                navigation_state = rerouting_state;
            }
            heading_change += heading_increment;
            // safety measure in case it gets stuck
            if (heading_change > 180){
              set_target(find_node(route.list_nodes, next_wp_route + 'a'));
            }
            increase_nav_heading(heading_increment);

            // make sure we have a couple of good readings before declaring the way safe

            break;

        case out_of_bounds_state: //3
            increase_nav_heading(heading_increment);
            moveWaypointForward(WP_TRAJECTORY, 1.f);

            if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                // add offset to head back into arena
//                increase_nav_heading(heading_increment);

                // reset safe counter
                obstacle_free_confidence = 0;

                // ensure direction is safe before continuing
                navigation_state = search_heading_state;
            }
            break;
    }
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
        return false;
    }

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
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
uint8_t chooseRandomIncrementAvoidance(int failure_case)
{
    // Randomly choose CW or CCW avoiding direction
    if (failure_case == 1){
      if (orange_y > 0){
        heading_increment = 15.f;
      } else {
        heading_increment = -15.f;
      }
    } else {
      if (rand() % 2) {
        heading_increment = 15.f;
      } else {
        heading_increment = -15.f;
      }
      return false;
    }
}

int rotate(int change_heading){
    if (rand() % 2) {
        heading_increment = change_heading;
    } else {
        heading_increment = change_heading;
    }
    return false;
}