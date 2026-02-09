#ifndef MIN_HEAP_H
#define MIN_HEAP_H

typedef struct {
    int node_id;
    double dist; 
} MinHeapNode;

typedef struct {
    int capacity;    
    int size;        
    int *pos;        // position of each node in the heap
    MinHeapNode **array; 
} MinHeap;


MinHeap* createMinHeap(int capacity);
MinHeapNode* newMinHeapNode(int node_id, double dist);
void swapMinHeapNode(MinHeapNode** a, MinHeapNode** b);
void minHeapify(MinHeap* minHeap, int idx);
int isEmpty(MinHeap* minHeap);
MinHeapNode* extractMin(MinHeap* minHeap);
void decreaseKey(MinHeap* minHeap, int node_id, double dist);
int isInMinHeap(MinHeap *minHeap, int node_id);
void freeMinHeap(MinHeap* minHeap);

#endif