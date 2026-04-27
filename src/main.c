#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "graph.h"
#include "graph_loader.h"
#include "server.h"

#ifndef ROUTE_WORKERS
#define ROUTE_WORKERS 8
#endif

int main(int argc, char* argv[]) {
    int route_workers = ROUTE_WORKERS;
    int port = 8080;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--workers") && i + 1 < argc)
            route_workers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--port") && i + 1 < argc)
            port = atoi(argv[++i]);
    }

    Graph* g = (Graph*)malloc(sizeof(Graph));
    if (!g) {
        fprintf(stderr, "Failed to allocate graph\n");
        return 1;
    }

    printf("MAIN: loading graph...\n");
    int rc = graph_load_from_files(g, "data/graph.meta", "data/nodes.csv", "data/edges.csv");
    if (rc != 0) {
        fprintf(stderr, "Failed to load graph (rc=%d)\n", rc);
        free(g);
        return 1;
    }

    printf("MAIN: starting server on port %d with %d routing workers...\n", port, route_workers);
    rc = server_run(g, port, route_workers);

    graph_free(g);
    free(g);
    return rc;
}
