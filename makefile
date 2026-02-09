CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2
LDFLAGS = -lm -pthread

SRC = \
    src/main.c \
    src/server.c \
    src/graph_loader.c \
    src/graph.c \
    src/routing.c \
    src/min_heap.c

TARGET = server

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
