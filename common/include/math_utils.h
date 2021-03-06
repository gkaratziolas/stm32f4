#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include "stm32f4xx.h"
#include "arm_math.h"

struct int32_vec {
        union {
                int32_t a;
                int32_t x;
        };
        union {
                int32_t b;
                int32_t y;
        };
};

struct float32_vec {
        union {
                float32_t a;
                float32_t x;
        };
        union {
                float32_t b;
                float32_t y;
        };
};

int32_t int32_abs(int32_t A);
float32_t float32_abs(float32_t A);

struct int32_vec int32_vec_add(struct int32_vec *A, struct int32_vec *B);
struct int32_vec int32_vec_sub(struct int32_vec *A, struct int32_vec *B);
int64_t int32_vec_mag_squared(struct int32_vec *A);

struct float32_vec float32_vec_add(struct float32_vec *A, struct float32_vec *B);
struct float32_vec float32_vec_sub(struct float32_vec *A, struct float32_vec *B);
float32_t float32_vec_mag(struct float32_vec *A);
float32_t float32_vec_mag_squared(struct float32_vec *A);

float32_t ccw_angle(struct float32_vec *A, struct float32_vec *B);
float32_t cw_angle(struct float32_vec *A, struct float32_vec *B);

#endif // MATH_UTILS_H_
