#include "math_utils.h"
#include <math.h>

#ifdef DEBUG
#include "debug_usart.h"
#endif //DEBUG_

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

float32_t float32_vec_mag(struct float32_vec *A)
{
        float32_t mag_squared = float32_vec_mag_squared(A);
        // TODO: implement real sqrt!
        if (float32_abs(A->x) >= float32_abs(A->y))
                return mag_squared / float32_abs(A->x);
        return mag_squared / float32_abs(A->y);
}

float32_t float32_vec_mag_squared(struct float32_vec *A)
{
        float32_t mag_squared;
        mag_squared = A->x * A->x + A->y * A->y;
        return mag_squared;
}

int32_t int32_abs(int32_t A)
{
        if (A >= 0)
                return A;
        return -1 * A;
}

float32_t float32_abs(float32_t A)
{
        if (A >= 0)
                return A;
        return -1 * A;
}

// Returns counter-clockwise angle from vec a to vec b. (2*PI > theta >= 0)
float32_t ccw_angle(struct float32_vec *A, struct float32_vec *B)
{
        // calculate determinant
        float32_t alpha = A->x * B->y - A->y * B->x;
        
        // calculate dot product
        float32_t beta  = A->x * B->x + A->y * B->y;

        // calculate cw angle (PI >= angle > -PI )
        float32_t angle = atan2f(alpha, beta);

        if (angle >= 0)
                return angle;
        // If angle is negative, it represents the clockwise rotation.
        // Add to 2PI and return
        return 2*PI + angle;    
}

// Returns clockwise angle from vec a to vec b. (2*PI > theta >= 0)
float32_t cw_angle(struct float32_vec *A, struct float32_vec *B)
{
        // calculate determinant
        float32_t alpha = A->x * B->y - A->y * B->x;
        
        // calculate dot product
        float32_t beta  = A->x * B->x + A->y * B->y;

        // calculate cw angle (PI >= angle > -PI )
        float32_t angle = atan2f(alpha, beta);

        // Negative angle represents clockwise rotation. Flip sign and return
        if (angle <= 0)
                return -1 * angle;
        return 2*PI - angle;
}
