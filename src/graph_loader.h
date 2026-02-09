#ifndef GRAPH_LOADER_H
#define GRAPH_LOADER_H

#include "graph.h"

/**
 * Loads the graph from:
 *   meta_path:  contains "num_nodes N" and "num_edges M"
 *   nodes_path: CSV: node_id,x,y
 *   edges_path: CSV: edge_id,from_node,to_node,base_length,base_speed_limit
 *
 * Returns 0 on success, non-zero on error.
 */
int graph_load_from_files(Graph* g,
                          const char* meta_path,
                          const char* nodes_path,
                          const char* edges_path);

#endif
