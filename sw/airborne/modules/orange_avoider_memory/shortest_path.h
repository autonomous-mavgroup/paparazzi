//
// Created by lorenzo on 11-3-19.
//

#ifndef GRAPH_WAYPOINTS_SHORTEST_PATH_H
#define GRAPH_WAYPOINTS_SHORTEST_PATH_H
#include "graph.h"

typedef struct {
    int *data;
    int *prio;
    int *index;
    int len;
    int size;
} heap_t;

typedef struct Path {
    int dist;
    char* path;
    int number_nodes;
} Path;

heap_t *create_heap (int n);
void push_heap (heap_t *h, int v, int p);
int pop_heap (heap_t *h);
void dijkstra (graph_t *g, int a, int b);
void block_edge(graph_t* g, int a, int b);
Path get_path(graph_t *g, int i);
void print_path(Path*);

#endif //GRAPH_WAYPOINTS_SHORTEST_PATH_H
