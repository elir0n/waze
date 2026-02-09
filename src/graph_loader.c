#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "graph_loader.h"

/* ---- helpers ---- */

static void dief(const char* msg, const char* path) {
    fprintf(stderr, "ERROR: %s: %s\n", msg, path ? path : "(null)");
}

static int read_meta_counts(const char* meta_path, int* out_nodes, int* out_edges) {
    FILE* f = fopen(meta_path, "r");
    if (!f) {
        dief("failed to open meta file", meta_path);
        return 1;
    }

    int num_nodes = -1, num_edges = -1;
    char key[64];
    int val;

    while (fscanf(f, "%63s %d", key, &val) == 2) {
        if (strcmp(key, "num_nodes") == 0) num_nodes = val;
        else if (strcmp(key, "num_edges") == 0) num_edges = val;
        /* ignore unknown keys */
    }

    fclose(f);

    if (num_nodes <= 0 || num_edges < 0) {
        fprintf(stderr, "ERROR: meta file missing/invalid counts (num_nodes=%d, num_edges=%d)\n",
                num_nodes, num_edges);
        return 2;
    }

    *out_nodes = num_nodes;
    *out_edges = num_edges;
    return 0;
}

/* read a line, stripping trailing newline */
static int read_line(FILE* f, char* buf, size_t cap) {
    if (!fgets(buf, (int)cap, f)) return 0;
    size_t n = strlen(buf);
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) {
        buf[n-1] = '\0';
        n--;
    }
    return 1;
}

/* ---- public API ---- */

int graph_load_from_files(Graph* g,
                          const char* meta_path,
                          const char* nodes_path,
                          const char* edges_path)
{
    if (!g || !meta_path || !nodes_path || !edges_path) {
        fprintf(stderr, "ERROR: graph_load_from_files: NULL argument\n");
        return 10;
    }

    int num_nodes = 0, num_edges = 0;
    int rc = read_meta_counts(meta_path, &num_nodes, &num_edges);
    if (rc != 0) return rc;

    /* Initialize graph */
    graph_init(g, num_nodes, num_edges);

    /* ---- load nodes ---- */
    FILE* fn = fopen(nodes_path, "r");
    if (!fn) {
        dief("failed to open nodes file", nodes_path);
        graph_free(g);
        return 20;
    }

    char line[512];

    /* skip header (first line) */
    if (!read_line(fn, line, sizeof(line))) {
        fprintf(stderr, "ERROR: nodes file is empty\n");
        fclose(fn);
        graph_free(g);
        return 21;
    }

    int loaded_nodes = 0;
    while (read_line(fn, line, sizeof(line))) {
        if (line[0] == '\0') continue;

        int node_id;
        double x, y;

        /* expected: node_id,x,y */
        if (sscanf(line, "%d,%lf,%lf", &node_id, &x, &y) != 3) {
            fprintf(stderr, "ERROR: bad nodes.csv line: '%s'\n", line);
            fclose(fn);
            graph_free(g);
            return 22;
        }

        if (node_id < 0 || node_id >= g->num_nodes) {
            fprintf(stderr, "ERROR: node_id out of range: %d\n", node_id);
            fclose(fn);
            graph_free(g);
            return 23;
        }

        graph_set_node_coordinates(g, node_id, x, y);
        loaded_nodes++;
    }

    fclose(fn);

    /* ---- load edges ---- */
    FILE* fe = fopen(edges_path, "r");
    if (!fe) {
        dief("failed to open edges file", edges_path);
        graph_free(g);
        return 30;
    }

    /* skip header */
    if (!read_line(fe, line, sizeof(line))) {
        fprintf(stderr, "ERROR: edges file is empty\n");
        fclose(fe);
        graph_free(g);
        return 31;
    }

    int loaded_edges = 0;
    while (read_line(fe, line, sizeof(line))) {
        if (line[0] == '\0') continue;

        int edge_id, from, to;
        double len, speed;

        /* expected: edge_id,from_node,to_node,base_length,base_speed_limit */
        if (sscanf(line, "%d,%d,%d,%lf,%lf",
                   &edge_id, &from, &to, &len, &speed) != 5) {
            fprintf(stderr, "ERROR: bad edges.csv line: '%s'\n", line);
            fclose(fe);
            graph_free(g);
            return 32;
        }

        if (edge_id < 0 || edge_id >= g->num_edges) {
            fprintf(stderr, "ERROR: edge_id out of range: %d\n", edge_id);
            fclose(fe);
            graph_free(g);
            return 33;
        }

        graph_add_edge(g, edge_id, from, to, len, speed);
        loaded_edges++;
    }

    fclose(fe);

    if (loaded_edges != g->num_edges) {
        fprintf(stderr, "ERROR: edges count mismatch (expected %d, got %d)\n",
                g->num_edges, loaded_edges);
        graph_free(g);
        return 34;
    }

    return 0;
}


    
