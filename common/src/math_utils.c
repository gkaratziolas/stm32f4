#include "math_utils.h"

struct int32_vec int32_vec_add(struct int32_vec *A, struct int32_vec *B)
{
        struct int32_vec C;
        C.x = A->x + B->x;
        C.y = A->y + B->y;
        return C;
}

struct int32_vec int32_vec_sub(struct int32_vec *A, struct int32_vec *B)
{
        struct int32_vec C;
        C.x = A->x - B->x;
        C.y = A->y - B->y;
        return C;
}

int64_t int32_vec_mag_squared(struct int32_vec *A)
{
        int64_t mag_squared;
        mag_squared = (int64_t)A->x * (int64_t)A->x +
                      (int64_t)A->y * (int64_t)A->y;
        return mag_squared;
}

struct float32_vec float32_vec_add(struct float32_vec *A, struct float32_vec *B)
{
        struct float32_vec C;
        C.x = A->x + B->x;
        C.y = A->y + B->y;
        return C;
}

struct float32_vec float32_vec_sub(struct float32_vec *A, struct float32_vec *B)
{
        struct float32_vec C;
        C.x = A->x - B->x;
        C.y = A->y - B->y;
        return C;
}

float32_t float32_vec_mag_squared(struct float32_vec *A)
{
        float32_t mag_squared;
        mag_squared = (int64_t)A->x * (int64_t)A->x +
                      (int64_t)A->y * (int64_t)A->y;
        return mag_squared;
}

int32_t int_abs(int32_t a)
{
        if (a > 0) {
                return a;
        }
        return -1*a;
}
