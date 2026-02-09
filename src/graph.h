#ifndef GRAPH_H
#define GRAPH_H

#include <stdlib.h>

#define MAX_NODES 100000

typedef struct {
    int edge_id;
    int from_node;
    int to_node;

    double base_length;
    double base_speed_limit;

    double current_travel_time;

    // Historical statistics (for traffic updates / prediction)
    double ema_travel_time;
    int observation_count;
} Edge;

typedef struct EdgeNode {
    int edge_id;
    struct EdgeNode* next;
} EdgeNode;

typedef struct {
    int node_id;
    double x;
    double y;
    EdgeNode* out_edges;
} Node;

typedef struct {
    Node nodes[MAX_NODES];
    Edge* edges;

    int num_nodes;
    int num_edges;
} Graph;

/* Graph API */
void graph_init(Graph* g, int num_nodes, int num_edges);
void graph_add_edge(Graph* g, int edge_id, int from, int to,
                    double length, double speed_limit);

double get_edge_weight(Graph* g, int edge_id);
double heuristic(Graph* g, int from_node, int to_node);
void graph_set_node_coordinates(Graph* g, int node_id, double x, double y);
void graph_free(Graph* g);

#endif
