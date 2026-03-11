# Stage 1: build the C routing server
FROM debian:bookworm-slim AS builder
RUN apt-get update && apt-get install -y --no-install-recommends build-essential && rm -rf /var/lib/apt/lists/*
WORKDIR /build
COPY src/ src/
COPY makefile .
RUN make

# Stage 2: runtime image with Python + compiled binary
FROM python:3.13-slim
WORKDIR /app

COPY --from=builder /build/server ./server

COPY flow_field/ flow_field/
COPY gui/ gui/
COPY scripts/ scripts/
COPY *.py ./

RUN pip install --no-cache-dir numpy osmnx \
 && chmod +x /app/server

EXPOSE 8080 8090
