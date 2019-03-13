//
// Created by lorenzo on 11-3-19.
//

#ifndef GRAPH_WAYPOINTS_GRAPH_H
#define GRAPH_WAYPOINTS_GRAPH_H

typedef struct {
    int vertex;
    float weight;
} edge_t;

typedef struct {
    edge_t **edges;
    int edges_len;
    int edges_size;
    int dist;
    int prev;
    int visited;
    float x;
    float y;
} vertex_t;

typedef struct {
    vertex_t **vertices;
    int vertices_len;
    int vertices_size;
} graph_t;


void add_vertex (graph_t *g, int i);
void add_edge (graph_t *g, int a, int b, int w);
void add_vertex_coordinates(graph_t* g, int name, float x, float y);
void add_double_edge(graph_t* g, int a, int b, int weight);
void print_vertex(graph_t*, int);
#endif //GRAPH_WAYPOINTS_GRAPH_H
