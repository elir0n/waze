#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <stdint.h>

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

static char* trim_ws(char* s) {
    while (*s && isspace((unsigned char)*s)) s++;
    if (*s == '\0') return s;

    char* end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
    return s;
}

static int parse_int_strict(const char* s, int* out) {
    if (!s || !out) return 1;
    errno = 0;
    char* end = NULL;
    long v = strtol(s, &end, 10);
    if (s == end || errno != 0) return 1;
    while (end && *end) {
        if (!isspace((unsigned char)*end)) return 1;
        end++;
    }
    if (v < INT_MIN || v > INT_MAX) return 1;
    *out = (int)v;
    return 0;
}

static int parse_ll_strict(const char* s, long long* out) {
    if (!s || !out) return 1;
    errno = 0;
    char* end = NULL;
    long long v = strtoll(s, &end, 10);
    if (s == end || errno != 0) return 1;
    while (end && *end) {
        if (!isspace((unsigned char)*end)) return 1;
        end++;
    }
    *out = v;
    return 0;
}

static int parse_double_strict(const char* s, double* out) {
    if (!s || !out) return 1;
    errno = 0;
    char* end = NULL;
    double v = strtod(s, &end);
    if (s == end || errno != 0) return 1;
    while (end && *end) {
        if (!isspace((unsigned char)*end)) return 1;
        end++;
    }
    *out = v;
    return 0;
}

/* CSV splitter that handles double-quoted fields containing commas.
   Quotes are stripped from the field value in-place. */
static int split_csv_fields(char* line, char** fields, int max_fields) {
    int count = 0;
    char* p = line;

    while (count < max_fields) {
        if (*p == '"') {
            /* Quoted field: strip opening quote, find closing quote */
            p++;
            fields[count++] = p;
            char* close = strchr(p, '"');
            if (close) {
                *close = '\0';
                p = close + 1;
                if (*p == ',') p++;
            }
        } else {
            fields[count++] = p;
            char* comma = strchr(p, ',');
            if (!comma) break;
            *comma = '\0';
            p = comma + 1;
        }
    }

    return count;
}

static int skip_header_row(FILE* f, char* line, size_t line_cap) {
    while (read_line(f, line, line_cap)) {
        char* t = trim_ws(line);
        if (*t == '\0') continue;
        return 0;
    }
    return 1;
}

/* ---- node ID remapping ---- */

/* The CSV files use arbitrary OSM node IDs (potentially > INT_MAX).
   The graph internally uses sequential indices 0..num_nodes-1.
   We build a sorted table of (raw_osm_id -> sequential_index) on first
   pass through nodes.csv, then look up from/to IDs when loading edges. */

typedef struct {
    long long raw_id;
    int       index;
} NodeMapping;

static int node_mapping_cmp(const void* a, const void* b) {
    long long ra = ((const NodeMapping*)a)->raw_id;
    long long rb = ((const NodeMapping*)b)->raw_id;
    if (ra < rb) return -1;
    if (ra > rb) return  1;
    return 0;
}

static int lookup_node_index(const NodeMapping* map, int n, long long raw_id) {
    int lo = 0, hi = n - 1;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        if (map[mid].raw_id == raw_id) return map[mid].index;
        if (map[mid].raw_id < raw_id)  lo = mid + 1;
        else                            hi = mid - 1;
    }
    return -1;
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

    /* ---- load nodes (first pass: assign sequential indices + set coords) ---- */
    FILE* fn = fopen(nodes_path, "r");
    if (!fn) {
        dief("failed to open nodes file", nodes_path);
        graph_free(g);
        return 20;
    }

    char line[512];

    /* skip header */
    if (skip_header_row(fn, line, sizeof(line)) != 0) {
        fprintf(stderr, "ERROR: nodes file is empty\n");
        fclose(fn);
        graph_free(g);
        return 21;
    }

    NodeMapping* nmap = (NodeMapping*)malloc(sizeof(NodeMapping) * (size_t)num_nodes);
    if (!nmap) {
        fprintf(stderr, "ERROR: failed to allocate node mapping table\n");
        fclose(fn);
        graph_free(g);
        return 25;
    }

    int loaded_nodes = 0;
    while (read_line(fn, line, sizeof(line))) {
        char* tline = trim_ws(line);
        if (*tline == '\0') continue;

        char* fields[4];
        int field_count = split_csv_fields(tline, fields, 4);
        if (field_count != 3) {
            fprintf(stderr, "ERROR: bad nodes.csv line: '%s'\n", tline);
            fclose(fn);
            free(nmap);
            graph_free(g);
            return 22;
        }

        char* node_id_s = trim_ws(fields[0]);
        char* lat_s = trim_ws(fields[1]);
        char* lon_s = trim_ws(fields[2]);

        long long raw_id;
        double lat, lon;
        if (parse_ll_strict(node_id_s, &raw_id) != 0 ||
            parse_double_strict(lat_s, &lat) != 0 ||
            parse_double_strict(lon_s, &lon) != 0) {
            fprintf(stderr, "ERROR: bad nodes.csv line: '%s'\n", tline);
            fclose(fn);
            free(nmap);
            graph_free(g);
            return 22;
        }

        if (loaded_nodes >= num_nodes) {
            fprintf(stderr, "ERROR: more nodes in CSV than declared in meta (%d)\n", num_nodes);
            fclose(fn);
            free(nmap);
            graph_free(g);
            return 23;
        }

        /* Assign sequential index = order of appearance */
        int idx = loaded_nodes;
        nmap[idx].raw_id = raw_id;
        nmap[idx].index  = idx;

        graph_set_node_coordinates(g, idx, lat, lon);
        loaded_nodes++;
    }

    fclose(fn);

    if (loaded_nodes != g->num_nodes) {
        fprintf(stderr, "ERROR: nodes count mismatch (expected %d, got %d)\n",
                g->num_nodes, loaded_nodes);
        free(nmap);
        graph_free(g);
        return 24;
    }

    /* Sort mapping table so edges can binary-search by raw OSM id */
    qsort(nmap, (size_t)num_nodes, sizeof(NodeMapping), node_mapping_cmp);

    /* ---- load edges ---- */
    FILE* fe = fopen(edges_path, "r");
    if (!fe) {
        dief("failed to open edges file", edges_path);
        free(nmap);
        graph_free(g);
        return 30;
    }

    /* skip header */
    if (skip_header_row(fe, line, sizeof(line)) != 0) {
        fprintf(stderr, "ERROR: edges file is empty\n");
        fclose(fe);
        free(nmap);
        graph_free(g);
        return 31;
    }

    int loaded_edges = 0;
    while (read_line(fe, line, sizeof(line))) {
        char* tline = trim_ws(line);
        if (*tline == '\0') continue;

        char* fields[9];
        int field_count = split_csv_fields(tline, fields, 9);
        if (field_count != 8) {
            fprintf(stderr, "ERROR: bad edges.csv line: '%s'\n", tline);
            fclose(fe);
            free(nmap);
            graph_free(g);
            return 32;
        }

        char* edge_id_s   = trim_ws(fields[0]);
        char* from_s      = trim_ws(fields[1]);
        char* to_s        = trim_ws(fields[2]);
        char* len_s       = trim_ws(fields[3]);
        char* speed_s     = trim_ws(fields[4]);
        char* road_type_s = trim_ws(fields[5]);
        char* lanes_s     = trim_ws(fields[6]);
        char* is_oneway_s = trim_ws(fields[7]);

        int edge_id, lanes, is_oneway;
        long long raw_from, raw_to;
        double len, speed;
        if (parse_int_strict(edge_id_s, &edge_id) != 0 ||
            parse_ll_strict(from_s, &raw_from) != 0 ||
            parse_ll_strict(to_s, &raw_to) != 0 ||
            parse_double_strict(len_s, &len) != 0 ||
            parse_double_strict(speed_s, &speed) != 0 ||
            parse_int_strict(lanes_s, &lanes) != 0 ||
            parse_int_strict(is_oneway_s, &is_oneway) != 0) {
            fprintf(stderr, "ERROR: bad edges.csv line: '%s'\n", tline);
            fclose(fe);
            free(nmap);
            graph_free(g);
            return 32;
        }

        if (edge_id < 0 || edge_id >= g->num_edges) {
            fprintf(stderr, "ERROR: edge_id out of range: %d\n", edge_id);
            fclose(fe);
            free(nmap);
            graph_free(g);
            return 33;
        }

        int from = lookup_node_index(nmap, num_nodes, raw_from);
        int to   = lookup_node_index(nmap, num_nodes, raw_to);
        if (from < 0 || to < 0) {
            fprintf(stderr, "ERROR: edge %d references unknown node (%lld -> %lld)\n",
                    edge_id, raw_from, raw_to);
            fclose(fe);
            free(nmap);
            graph_free(g);
            return 35;
        }

        graph_add_edge(g, edge_id, from, to, len, speed, road_type_s, lanes, is_oneway);
        loaded_edges++;
    }

    fclose(fe);
    free(nmap);

    if (loaded_edges != g->num_edges) {
        fprintf(stderr, "ERROR: edges count mismatch (expected %d, got %d)\n",
                g->num_edges, loaded_edges);
        graph_free(g);
        return 34;
    }

    return 0;
}


    
