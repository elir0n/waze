#!/bin/bash
# Server entrypoint: download graph on first run, then start the C server.
set -e

DATA=/app/data

if [ ! -f "$DATA/graph.meta" ]; then
    echo "[init] No graph data found. Downloading Tel Aviv from OpenStreetMap (one-time, ~1-2 min)..."
    python3 /app/scripts/download_tel_aviv.py --out "$DATA"
    echo "[init] Done. Starting server..."
fi

exec /app/server
