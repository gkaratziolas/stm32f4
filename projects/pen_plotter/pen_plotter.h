#ifndef PEN_PLOTTER_H_
#define PEN_PLOTTER_H_

#include "stm32f4xx.h"
#include "math_utils.h"

// Default velocity curve values for tmc5041 motor driver
#define MAX_A1            0x000013E8
#define MAX_V1            0x0001c350
#define MAX_AMAX          0x0000FFFF
#define MAX_VMAX          0x001304d0
#define MAX_DMAX          0x000012bc
#define MAX_D1            0x00001578
#define MAX_VSTOP         0x0000000A

#define RAMPMODE_POSITION 0x00000000
#define RAMPMODE_VPOS     0x00000001
#define RAMPMODE_VNEG     0x00000002
#define RAMPMODE_HOLD     0x00000003

// Functions for moving pen head
void pen_motors_init(void);
void pen_set_motor_motion_params_max(int motor);
uint8_t pen_set_velocity(struct float32_vec velocity);
uint8_t pen_stop(void);
uint8_t pen_set_target_position(struct int32_vec target);
struct int32_vec pen_get_position(void);
uint32_t pen_motion_check_complete(struct int32_vec start, struct int32_vec end);

#endif // PEN_PLOTTER_H_
