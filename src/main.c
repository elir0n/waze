#include <stdio.h>
#include <stdlib.h>
#include "graph.h"
#include "graph_loader.h"
#include "server.h"

int main(void) {
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

    /* starts server on port 8080 */
    rc = server_run(g, 8080);

    graph_free(g);
    free(g);
    return rc;
}
