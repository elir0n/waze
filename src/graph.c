#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "graph.h"

void graph_init(Graph* g, int num_nodes, int num_edges)
{
    if (!g) {
        fprintf(stderr, "graph_init: graph is NULL\n");
        exit(1);
    }

    if (num_nodes > MAX_NODES) {
        fprintf(stderr, "graph_init: num_nodes exceeds MAX_NODES\n");
        exit(1);
    }

    g->num_nodes = num_nodes;
    g->num_edges = num_edges;

    /* Allocate global edge table */
    if (num_edges > 0) {
    g->edges = (Edge*)malloc(sizeof(Edge) * num_edges);
    if (!g->edges) {
        fprintf(stderr, "graph_init: failed to allocate edges array\n");
        exit(1);
    }
    } else {
        g->edges = NULL;
    }

    /* Initialize nodes */
    for (int i = 0; i < num_nodes; i++) {
        g->nodes[i].node_id = i;
        g->nodes[i].x = 0.0;
        g->nodes[i].y = 0.0;
        g->nodes[i].out_edges = NULL;
    }
}


void graph_set_node_coordinates(Graph* g, int node_id, double x, double y)
{
    if (!g || node_id < 0 || node_id >= g->num_nodes) {
        fprintf(stderr, "graph_set_node_coordinates: invalid node_id\n");
        exit(1);
    }

    g->nodes[node_id].x = x;
    g->nodes[node_id].y = y;
}


void graph_add_edge(Graph* g,
                    int edge_id,
                    int from,
                    int to,
                    double length,
                    double speed_limit)
{
    if (!g) {
        fprintf(stderr, "graph_add_edge: graph is NULL\n");
        exit(1);
    }

    if (edge_id < 0 || edge_id >= g->num_edges) {
        fprintf(stderr, "graph_add_edge: invalid edge_id %d\n", edge_id);
        exit(1);
    }

    if (from < 0 || from >= g->num_nodes ||
        to   < 0 || to   >= g->num_nodes) {
        fprintf(stderr, "graph_add_edge: invalid node ids (%d -> %d)\n",
                from, to);
        exit(1);
    }

    if (speed_limit <= 0.0) {
        fprintf(stderr, "graph_add_edge: speed_limit must be positive\n");
        exit(1);
    }

    /* Initialize global edge */
    Edge* e = &g->edges[edge_id];

    e->edge_id = edge_id;
    e->from_node = from;
    e->to_node = to;

    e->base_length = length;
    e->base_speed_limit = speed_limit;

    /* Initial travel time */
    e->current_travel_time = length / speed_limit;

    /* Initialize historical stats */
    e->ema_travel_time = e->current_travel_time;
    e->observation_count = 0;

    /* Add edge_id to adjacency list of 'from' node */
    EdgeNode* node = (EdgeNode*)malloc(sizeof(EdgeNode));
    if (!node) {
        fprintf(stderr, "graph_add_edge: malloc failed\n");
        exit(1);
    }

    node->edge_id = edge_id;
    node->next = g->nodes[from].out_edges;
    g->nodes[from].out_edges = node;
}


double get_edge_weight(Graph* g, int edge_id)
{
    if (!g || edge_id < 0 || edge_id >= g->num_edges) {
        fprintf(stderr, "get_edge_weight: invalid edge_id\n");
        exit(1);
    }

    return g->edges[edge_id].current_travel_time;
}


double heuristic(Graph* g, int from_node, int to_node)
{
    if (!g ||
        from_node < 0 || from_node >= g->num_nodes ||
        to_node   < 0 || to_node   >= g->num_nodes) {
        fprintf(stderr, "heuristic: invalid node ids\n");
        exit(1);
    }

    double dx = g->nodes[from_node].x - g->nodes[to_node].x;
    double dy = g->nodes[from_node].y - g->nodes[to_node].y;
    double straight_dist = sqrt(dx * dx + dy * dy);

    /* Use a time-based admissible heuristic: straight-line distance / max speed */
    double max_speed = 0.0;
    for (int i = 0; i < g->num_edges; i++) {
        if (g->edges && g->edges[i].base_speed_limit > max_speed) {
            max_speed = g->edges[i].base_speed_limit;
        }
    }

    if (max_speed > 0.0) {
        return straight_dist / max_speed;
    }

    /* No speed info; fall back to distance */
    return straight_dist;
}


void graph_free(Graph* g)
{
    if (!g) return;

    for (int i = 0; i < g->num_nodes; i++) {
        EdgeNode* curr = g->nodes[i].out_edges;
        while (curr) {
            EdgeNode* tmp = curr;
            curr = curr->next;
            free(tmp);
        }
        g->nodes[i].out_edges = NULL;
    }

    free(g->edges);
    g->edges = NULL;
}
