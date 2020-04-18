#include "fifo.h"

// string header reqired for memcpy
#include <string.h>

/*
 * to avoid use of malloc, an emptpy array pointer for a predefined 
 * array of the wanted length must be passed to this function. e.g.
 * #define LEN 100;
 * int array[LEN];
 * stuct fifo my_fifo = fifo_init(array, sizeof(int), LEN);
 */
struct fifo fifo_init(void *array, size_t element_size, int length)
{
        struct fifo f = {
                .array        = array,
                .element_size = element_size,
                .length       = length,
                .front        = 0,
                .back         = 0,
                .full         = 0,
        };

        if (f.length == 0) {
                f.full = 1;
        }

        return f;
}

int fifo_push(struct fifo *f, void *data)
{
        if (fifo_full(f))
                return FIFO_ERR_FULL;

        memcpy(f->array + (f->back * f->element_size), data, f->element_size);

        f->back++;
        if (f->back >= f->length)
                f->back = 0;

        if (f->back == f->front)
                f->full = 1;

        return FIFO_OK;
}

int fifo_pop(struct fifo *f, void *data)
{
        if (fifo_empty(f))
                return FIFO_ERR_EMPTY;
        
        memcpy(data, f->array + (f->front * f->element_size), f->element_size);
        
        f->front++;
        if (f->front >= f->length)
                f->front = 0;

        f->full = 0;
        return FIFO_OK;
}

int fifo_peek(struct fifo *f, void *data)
{
        if (fifo_empty(f))
                return FIFO_ERR_EMPTY;

        memcpy(data, f->array + (f->front * f->element_size), f->element_size);
        return FIFO_OK;
}

int fifo_full(struct fifo *f)
{
        return f->full;
}

int fifo_empty(struct fifo *f)
{
        return ((!f->full) && (f->front == f->back));
}

int fifo_space(struct fifo *f)
{
        if (fifo_empty(f))
                return f->length;
        if (f->front > f->back)
                return f->front - f->back;
        return f->length - f->back + f->front;
}
