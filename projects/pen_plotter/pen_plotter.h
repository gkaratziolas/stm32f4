#ifndef PEN_PLOTTER_H_
#define PEN_PLOTTER_H_

#include "stm32f4xx.h"
#include "math_utils.h"
#include "fifo.h"

/*
 ******************************
 * tmc5041 default parameters *
 ******************************
 */

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

// Square of distance threshold to mark coincidence 
#define DIST_SQUARED_TARGET_THRESHOLD (int64_t)200000

#define MAX_MOTIONS              1024
#define MAX_MOTIONS_PER_GCOMMAND  255

/*
 ********************************
 * public pen plotter functions *
 ********************************
 */

void pen_init(struct fifo *g);
void pen_deinit(void);
void pen_serve(void);

#endif // PEN_PLOTTER_H_
