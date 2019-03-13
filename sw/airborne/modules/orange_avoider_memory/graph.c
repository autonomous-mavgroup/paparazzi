//
// Created by lorenzo on 11-3-19.
//

#include "graph.h"

#include <stdio.h>
#include <stdlib.h>

void add_vertex (graph_t *g, int i) {
    if (g->vertices_size < i + 1) {
        int size = g->vertices_size * 2 > i ? g->vertices_size * 2 : i + 4;
        g->vertices = realloc(g->vertices, size * sizeof (vertex_t *));
        for (int j = g->vertices_size; j < size; j++)
            g->vertices[j] = NULL;
        g->vertices_size = size;
    }
    if (!g->vertices[i]) {
        g->vertices[i] = calloc(1, sizeof (vertex_t));
        g->vertices_len++;
    }
}

void add_edge (graph_t *g, int a, int b, int w) {
    a = a - 'a';
    b = b - 'a';
    add_vertex(g, a);
    add_vertex(g, b);
    vertex_t *v = g->vertices[a];
    if (v->edges_len >= v->edges_size) {
        v->edges_size = v->edges_size ? v->edges_size * 2 : 4;
        v->edges = realloc(v->edges, v->edges_size * sizeof (edge_t *));
    }
    edge_t *e = calloc(1, sizeof (edge_t));
    e->vertex = b;
    e->weight = w;
    v->edges[v->edges_len++] = e;
}

void add_double_edge(graph_t* g, int a, int b, int weight){
    add_edge(g, a, b, weight);
    add_edge(g, b, a, weight);
}

void add_vertex_coordinates(graph_t* g, int name, float x, float y){
    int id = name - 'a';
    g->vertices[id]->x = x;
    g->vertices[id]->y = y;
}

void print_vertex(graph_t* g, int name){
    int id = name - 'a';
    printf("Node %c: (%f, %f) \n", name, g->vertices[id]->x, g->vertices[id]->y);
    vertex_t* v = g->vertices[id];
    for (int i = 0; i < v->edges_len; i ++){
        printf("Edge %c - %c, dist : %f \n", name, v->edges[i]->vertex + 'a', v->edges[i]->weight);
    }
}

int id(int a){
    return a - 'a';
}
