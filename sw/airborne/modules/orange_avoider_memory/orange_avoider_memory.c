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

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif
uint8_t increase_nav_heading(float);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t chooseRandomIncrementAvoidance(void);
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

// define and initialise global variables
enum navigation_state_t navigation_state = safe_state;
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


void print_waypoint_pos(uint8_t wp){
    PRINT("Wp %i position (%i, %i) \n ", wp, 100*waypoint_get_x(wp), 100*waypoint_get_y(wp));
}

void print_point_pos(struct EnuCoor_i* point){
    PRINT("\n Point position : (%i, %i) ", point->x, point->y);
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
    PRINT("POSITION 1 : (%i, %i), Position 2 (%i, %i)", start_point->x, start_point->y, end_point->x, end_point->y);
    float distance = sqrt(powf(end_point->x - start_point->x, 2) + powf(end_point->y - start_point->y,2));
    PRINT("distance %i", distance);
    return distance;
}

void add_path(Route* route, Node point_1, Node point_2){
    // distance in centimeters
    route->list_nodes = add_node(route->list_nodes, point_1);
    route->list_nodes = add_node(route->list_nodes, point_2);
    int distance = distancePoints(&point_1.position, &point_2.position);
//    PRINT("distance %i", distance);
    add_double_edge(route->graph, point_1.name, point_2.name, distance);
}

void block_path(Route* route, Node* point_1, Node* point_2){
    block_edge(route->graph, point_1->name, point_2->name);
}

Path find_path(Route* route, Node* point_start, Node* point_end){
//    PRINT("FINDING PATH FROM %c to %c", point_start->name, point_end->name);
    dijkstra(route->graph, point_start->name, point_end->name);
//    PRINT("\nDIJKSRA RUNS\n");
    Path path = get_path(route->graph, point_end->name);
    return path;
}

void load_path(Route* route, Path* path){
//    PRINT("INSIDE \n");
    route->current_path = path;
    path->target_node = 1;
//    PRINT("Setting target \n");
    set_target(find_node(route->list_nodes, path->path[path->target_node]));
}

void load_next_wp(Route* route){
    route->current_path->target_node += 1;
    set_target(find_node(route->list_nodes, route->current_path->path[route->current_path->target_node]));
}

bool completed_path(Route* route){
    return route->current_path->target_node == route->current_path->number_nodes - 1;
}

struct EnuCoor_i get_wp_pos_enui(uint8_t wp_id){
    struct EnuCoor_i point;
    PRINT("\n X Y (%f, %f)\n", waypoints[wp_id].enu_f.x, waypoints[wp_id].enu_f.y);
    point.x = waypoints[wp_id].enu_i.x;
    point.y = waypoints[wp_id].enu_i.y;
    return point;
}

bool close_to_target(Node* target){
    return get_dist2_to_point(&target->position) < 0.5*0.5;
}

int select_rand_target(int avoid_this_num){
    int num = avoid_this_num;
    while( num == avoid_this_num){
        num = (rand() % 4) + 'a';
    }
    return num;
};

Node* get_target(Route* route){
    return find_node(route->list_nodes, route->current_path->target_node + 'a');
}

void set_target(Node* target){
//    PRINT("INSIDE");
//    PRINT( "(%i, %i)", target->position.x, target->position.x);
    navigation_target = target->position;
//    PRINT("\n NAVIGATION TARGET ASSIGNED \n");
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

void orange_avoider_init() {
    srand(time(NULL));
    // Initialize initial graph
    g = calloc(1, sizeof (graph_t));
    waypoints_init();
    route.graph = g;
    PRINT("FUNC X : %f \n", waypoint_get_x(WP__OZ1));
    struct EnuCoor_i pos_a = get_wp_pos_enui(WP__OZ1);
    PRINT("Position x y : (%i, %i)", pos_a.x, pos_a.y);
    Node a = {.name = 'a', .position = pos_a};
    struct EnuCoor_i pos_b = get_wp_pos_enui(WP__OZ2);
    Node b = {.name = 'b', .position = pos_b};
    struct EnuCoor_i pos_c = get_wp_pos_enui(WP__OZ3);
    Node c = {.name = 'c', .position = pos_c};
    struct EnuCoor_i pos_d = get_wp_pos_enui(WP__OZ4);
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
    PRINT("SELECTING RANDOM TARGET");
    target_node = *find_node(route.list_nodes, select_rand_target(-1));
    PRINT("\nnode name taraget %c\n", target_node.name);
    PRINT("\nFOUND TArget node\n");
    add_path(&route, last_node, target_node);
    PRINT("\nADDED TARGE PATH\n");
    current_path = find_path(&route, &last_node, &target_node);
    PRINT("\nGOING TO LOAD PATH\n");
    PRINT("\nFinished Initialiazation \n");
    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

void test_paths(){
    Node* a = find_node(route.list_nodes, 'a');
    Node* b= find_node(route.list_nodes, 'b');
    Node* c = find_node(route.list_nodes, 'c');
    Path path = find_path(route.graph, a->name, b->name);
    print_path(&path);
    block_edge(route.graph, a->name, c->name);
    Path new_path = find_path(route.graph, a->name, b->name);
    print_path(&new_path);
    for (int i = 0; i < 4; i++){
        print_waypoint_pos(corner_wps[i]);
    }
}

void orange_avoider_periodic() {
    struct EnuCoor_i wp_goal_pos = get_wp_pos_enui(WP_GOAL);
    PRINT("heading %i", nav_heading);
    // printf causes segmentation fautls apparently
    // only evaluate our state machine if we are flying
    if(!autopilot_in_flight()){
        return;
    }
    if (first_run){
        load_path(&route, &current_path);
        first_run = 0;
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
    PRINT("Obstacel free confidence %i", obstacle_free_confidence);
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

    float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

    // FMS implementation
    switch (navigation_state){
        case safe_state: //0
            PRINT("SAFE STATE \n");
            moveWaypointForward(WP_TRAJECTORY, 1.f * moveDistance);
            if (obstacle_free_confidence == 0){
                    navigation_state = obstacle_found_state;
            }
            print_waypoint_pos(WP_GOAL);
            print_point_pos(&target_node.position);
            PRINT("\n Distance to target : %f\n", get_dist2_to_point(&target_node));
            if (close_to_target(&target_node)){
                PRINT("CLOSE OT TARGET \n");
                if (completed_path(&route)){
                    last_node = target_node;
                    target_node = *find_node(route.list_nodes, select_rand_target(last_node.name));
                    current_path = find_path(&route, &last_node, &target_node);
                    load_path(&route, &current_path);
                } else {
                    load_next_wp(&route);
                }
            }
            break;

        case validate_node_state: //5
            {
            Node* deviation_node = get_target(&route); // still old target

            if (close_to_target(deviation_node) && obstacle_free_confidence > 3){
                target_node = *deviation_node;
                navigation_state = safe_state;
            } else{
                navigation_state = rerouting_state;
            }
            break;
        }
        case rerouting_state: //4

            moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
            if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                navigation_state = out_of_bounds_state;
            } else if (obstacle_free_confidence  <  1){
                navigation_state = obstacle_found_state;
            } else {
                moveWaypointForward(WP_GOAL, moveDistance);
                if (obstacle_free_confidence > 3){
                    Node deviation_node = {.name = get_new_node_name(route.list_nodes), .position = get_wp_pos_enui(WP_GOAL)};
//                    PRINT("\nLAST NODE %c, TARGET NODE %c, DEVIATION %c\n", last_node.name, target_node.name, deviation_node.name);
                    add_path(&route, last_node, deviation_node);
//                    PRINT("\nDISTANCE NODES last deviation %f\n", distancePoints(&last_node.position, &deviation_node.position));
//                    PRINT("\nDISTANCE NODES deviation target %f\n", distancePoints(&deviation_node.position, &target_node.position));
//                    PRINT("\nDISTANCE NODES last target %f \n", distancePoints(&last_node.position, &target_node.position));
                    add_path(&route, deviation_node, target_node);
//                    PRINT("\n deviation_ntion node edges %i \n", route.graph->vertices[deviation_node.name - 'a']->edges_len);
                    Node * ptr = find_node(route.list_nodes, deviation_node.name);
//                    PRINT("FOUDN NODE NAME %c", ptr->name);
                    block_edge(route.graph, last_node.name, target_node.name);
//                    PRINT("\nADDED PATH AND BLOCKED EDGE\n");
                    dijkstra(route.graph, last_node.name, target_node.name);
//                    PRINT("\n DIJSTRA has run \n");
                    current_path = find_path(&route, &last_node, &target_node);
                    print_path(&current_path);
//                    PRINT("\nLast node %c, target %c\n", last_node.name, target_node.name);
                    load_path(&route, &current_path);
                    navigation_state = validate_node_state;
//                    PRINT("LOADED PATH");
                }
            }
            break;

        case obstacle_found_state: //1
            PRINT("Obstacle found \n");
            waypoint_set_here_2d(WP_GOAL);
            waypoint_set_here_2d(WP_TRAJECTORY);

            // randomly select new search direction
            chooseRandomIncrementAvoidance();

            navigation_state = search_heading_state;
            break;

        case search_heading_state: //2
            increase_nav_heading(heading_increment);

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                navigation_state = rerouting_state;
            }
            break;

        case out_of_bounds_state: //3
            increase_nav_heading(heading_increment);
            moveWaypointForward(WP_TRAJECTORY, 1.5f);

            if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                // add offset to head back into arena
                increase_nav_heading(heading_increment);

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
