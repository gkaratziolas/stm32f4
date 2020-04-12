#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include "stm32f4xx.h"
#include "arm_math.h"

struct int32_vec {
        int32_t x;
        int32_t y;
};

struct float32_vec {
        float32_t x;
        float32_t y;
};

int32_t int_abs(int32_t a);

struct int32_vec int32_vec_add(struct int32_vec *A, struct int32_vec *B);
struct int32_vec int32_vec_sub(struct int32_vec *A, struct int32_vec *B);
int64_t int32_vec_mag_squared(struct int32_vec *A);

struct float32_vec float32_vec_add(struct float32_vec *A, struct float32_vec *B);
struct float32_vec float32_vec_sub(struct float32_vec *A, struct float32_vec *B);
float32_t float32_vec_mag_squared(struct float32_vec *A);

#endif // MATH_UTILS_H_