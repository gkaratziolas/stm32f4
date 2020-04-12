#ifndef FIFO_H_
#define FIFO_H_

#include <stdio.h>

struct fifo {
    void *array;
    int element_size;
    int length;
    int front;
    int back;
    int full;
};

struct fifo fifo_init(void *array, size_t element_size, int length);

int fifo_push(struct fifo *f, void *data); 
int fifo_pop(struct fifo *f, void *data);
int fifo_peek(struct fifo *f, void *data);

int fifo_full(struct fifo *f);
int fifo_empty(struct fifo *f);

#endif //FIFO_H_