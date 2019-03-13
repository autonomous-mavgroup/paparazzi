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

// States for Finite State Machine
enum navigation_state_t {
    safe_state,
    obstacle_found_state,
    search_heading_state,
    out_of_bounds_state,
    deviation_state
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = search_heading_state;
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


typedef struct Node{
    struct EnuCoor_i position;
    int name;
} Node;

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

ListNodes add_node(ListNodes head, Node node){
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
    return head;
}

typedef struct Route{
    graph_t* graph;
    ListNodes list_nodes;
} Route;


Node* find_node(ListNodes head, int node_name){
    ListNodes p;
    if (head == NULL){
        // empty list
        return NULL;
    }
    else{
        p = head;
        while(p->node.name != node_name){
            p = p->next;
        }
        if (p->node.name == node_name){return &(p->node);}
        else{
            // not found
            return NULL;
        }
    }
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

void block_path(graph_t* g, Node* point_1, Node* point_2){
    block_edge(g, point_1->name, point_2->name);
}

Path find_path(graph_t* g, int point_start_name, int point_end_name){
    dijkstra(g, point_start_name, point_end_name);
    Path path = get_path(g, point_end_name);
    return path;
}


struct EnuCoor_i wpToPoint(uint8_t wp_id){
    struct EnuCoor_i point;
    point.x = waypoints[wp_id].enu_i.x;
    point.y = waypoints[wp_id].enu_i.y;
    return point;
}

bool close_to_target(Node* target){
    return get_dist2_to_point(&target->position) < 0.5*0.5;
}

void set_target(Node* target){
    navigation_target = target->position;
    nav_set_heading_towards_target();
}

graph_t* g;
Route route;


void orange_avoider_init() {
    srand(time(NULL));
    // Initialize initial graph
    g = calloc(1, sizeof (graph_t));
    route.graph = g;
    struct EnuCoor_i pos_a = wpToPoint(WP__OZ1);
    Node a = {.name = 'a', .position = pos_a};
    struct EnuCoor_i pos_b =  wpToPoint(WP__OZ2);
    Node b = {.name = 'b', .position = pos_b};
    struct EnuCoor_i pos_c = wpToPoint(WP__OZ3);
    Node c = {.name = 'c', .position = pos_c};
    struct EnuCoor_i pos_d = wpToPoint(WP__OZ4);
    Node d = {.name = 'd', .position = pos_d};
    // Init Route
    // add connectivity
    add_path(&route, a, c);
    add_path(&route, a, d);
    add_path(&route, b, c);
    add_path(&route, b, d);
    PRINT("Initialization successful\n");
    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

void orange_avoider_periodic() {
    // printf causes segmentation fautls apparently
    Node* ptr = find_node(route.list_nodes, 'a');
    PRINT("node %c ", ptr->name);
    printf('d', ptr->name);
}


