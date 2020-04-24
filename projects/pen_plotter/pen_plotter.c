#include "pen_plotter.h"

#include "math_utils.h"
#include "tmc5041.h"
#include "gcode_reader.h"
// TODO: #include "servo_generic.h"

/*
 ****************************************
 * Private types for describing motions *
 ****************************************
 */

enum motion_type {
        MOTION_NONE,
        MOTION_XY,
        MOTION_Z,
};

struct motion {
        int type;
        union {
                struct int32_vec xy;
                uint8_t z;
        };
};

/*
 *****************************************
 * Private motion control loop functions *
 *****************************************
 */

/* Start motions */
void pen_motion_start(struct motion *target);
uint8_t pen_motion_start_XY(struct int32_vec target);
void pen_motion_start_Z(uint8_t target);

/* Check for motion complete */
int pen_motion_check_complete(struct motion target);
int pen_motion_check_complete_XY(struct motion target);
int pen_motion_check_complete_Z();

/* tmc5041 interface functions */
uint8_t pen_motors_init(void);
uint8_t pen_motor_set_motion_params_max(int motor);
uint8_t pen_get_position(struct int32_vec *position);
uint8_t pen_set_velocity(struct float32_vec velocity);
uint8_t pen_stop(void);

/* Servo interface functions */
// TODO!

/*
 **************************************
 * Static global variable definitions *
 **************************************
 */

/* Motion FIFO for storing goto xy / goto z commands */
#define MAX_MOTIONS 255
struct motion motions[MAX_MOTIONS];
struct fifo motion_fifo = fifo_init(motions, sizeof(struct motion), MAX_MOTIONS);

/*
 * Starting point for XY motions.
 * Referenced when checking motion complete.
 * Updated once XY motion is finished
 */
static struct int32_vec motion_XY_source;

/*
 ************************
 * Function definitions *
 ************************
 */

void pen_serve(void)
{
        static struct motion target; 
        if (!pen_motion_check_complete(&target)) {
                return;
        }
        if (fifo_pop(&motion_fifo, &target) == FIFO_ERR_EMPTY) {
                target.type = MOTION_NONE;
        }
        pen_motion_start(&target);
}

int pen_motion_check_complete(struct motion target);
{
        switch(target.type) {
        case MOTION_XY:
                if (pen_motion_check_complete_XY(motion_XY_source, &(target.xy)))) {
                        return 1;
                }
                return 0;
        case MOTION_Z:
                if (pen_motion_check_complete_Z(&(target.z))) {
                        return 1;
                }
                return 0;
        case MOTION_NONE:
                return 1;
        default:
                return 1; 
        }
}

int pen_motion_check_complete_XY(struct int32_vec *target)
{
        struct int32_vec position;
        pen_get_position(&position);

        struct int32_vec dist_to_start  = int32_vec_sub(&position, &motion_XY_source);
        struct int32_vec dist_to_end    = int32_vec_sub(&position, &target);
        struct int32_vec dist_start_end = int32_vec_sub(&end, &motion_XY_source);

        // If the pen is close to the target, or has travelled further than the
        // target from the start, then the motion is complete
        if ((int32_vec_mag_squared(&dist_to_start) > int32_vec_mag_squared(&dist_start_end)) |
            (int32_vec_mag_squared(&dist_to_end)   < DIST_SQUARED_TARGET_THRESHOLD)) {
                // update source position for future XY motions
                motion_XY_source.x = position.x;
                motion_XY_source.y = position.y;
                return 1;
        }
        return 0;
}

int pen_motion_check_complete_Z(struct int32_vec *target)
{
        // TODO: check servo position here
        return 1;
}

void pen_motion_start(struct motion *target)
{
        switch(target.type) {
        case MOTION_XY:
                pen_motion_start_XY(target->xy);
                break;
        case MOTION_Z:
                pen_motion_start_Z(target->z);
                break;
        case MOTION_NONE:
        default:
                pen_stop();
                break;
        }
}

uint8_t pen_motion_start_XY(struct int32_vec target)
{
        position = pen_get_position();
        struct int32_vec distance = int32_vec_sub(&target, &position);
        struct float32_vec velocity;

        if (int_abs(distance.x) >= int_abs(distance.y)) {
                if (distance.x > 0)
                        velocity.x = 1.0f;
                else
                        velocity.x = -1.0f;
                velocity.y = ((float32_t)distance.y * velocity.x) / (float32_t)distance.x;
        } else {
                if (distance.y > 0)
                        velocity.y = 1.0f;
                else
                        velocity.y = -1.0f;
                velocity.x = ((float32_t)distance.x * velocity.y) / (float32_t)distance.y;
        }
        
        motion_XY_source = position;
        return pen_set_velocity(velocity);
}

void pen_motion_start_Z(uint8_t target)
{
        // TODO: set servo position here 
        return;
}

uint8_t pen_motors_init()
{
        int i;
        uint8_t status = tmc5041_reg_write(tmc5041_GCONF, 0x00000008);

        // Initial motor config
        for (i=0; i<2; i++) {
                status |= tmc5041_reg_write(tmc5041_CHOPCONF[i],   0x000100c5);
                status |= tmc5041_reg_write(tmc5041_IHOLD_IRUN[i], 0x00011f05);
                status |= tmc5041_reg_write(tmc5041_TZEROWAIT[i],  0x00000000);
                status |= tmc5041_reg_write(tmc5041_PWMCONF[i],    0x000401c8);
                status |= tmc5041_reg_write(tmc5041_VHIGH[i],      0x00061a80);
                status |= tmc5041_reg_write(tmc5041_VCOOLTHRS[i],  0x00007530);

                // enable stallGuard2 stop
                // tmc5041_set_reg_bit(tmc5041_SW_MODE[i], 10, &status);
                // disable stallGuard2 filter
                // tmc5041_mask_reg(tmc5041_SW_MODE[i], ~(1<<24), &status);
                // set stallGuard2 threshold
                // uint32_t data = tmc5041_read_reg(tmc5041_COOLCONF, &status);
                // data = (data & 0xff00ffff) | (0x1 << 16);
                // tmc5041_write_reg(tmc5041_COOLCONF, data, &status);
                // tmc5041_mask_reg(tmc5041_COOLCONF[i], (0x40<<16), &status);
        }

        // Motor motion config
        for (i=0; i<2; i++) {
                status |= pen_set_motor_motion_params_max(i);
        }
        return status;
}

uint8_t pen_motor_set_motion_params_max(int motor)
{
        uint8_t status = 0;
        status |= tmc5041_reg_write(tmc5041_A1[motor],       MAX_A1           );
        status |= tmc5041_reg_write(tmc5041_V1[motor],       MAX_V1           );
        status |= tmc5041_reg_write(tmc5041_AMAX[motor],     MAX_AMAX         );
        status |= tmc5041_reg_write(tmc5041_VMAX[motor],     MAX_VMAX         );
        status |= tmc5041_reg_write(tmc5041_DMAX[motor],     MAX_DMAX         );
        status |= tmc5041_reg_write(tmc5041_D1[motor],       MAX_D1           );
        status |= tmc5041_reg_write(tmc5041_VSTOP[motor],    MAX_VSTOP        );
        status |= tmc5041_reg_write(tmc5041_RAMPMODE[motor], RAMPMODE_POSITION);
        return status;
}

uint8_t pen_get_position(struct int32_vec *position)
{
        uint8_t status = 0;
        status |= tmc5041_reg_read(tmc5041_XACTUAL[0], &(position->x));
        status |= tmc5041_reg_read(tmc5041_XACTUAL[1], &(position->y));
        return status;
}

// Set velocity yo some fraction of v_max between +/-1
uint8_t pen_set_velocity(struct float32_vec velocity)
{
        uint8_t status = 0;

        // Ensure velocity x and y are between +/-1 
        if (velocity.x > 1.0f)
                velocity.x = 1.0f;
        if (velocity.x < -1.0f)
                velocity.x = -1.0f;
        if (velocity.y > 1)
                velocity.y = 1;
        if (velocity.y < -1)
                velocity.y = -1;

        // Check whether velocity components are positive or negative
        if (velocity.x >= 0) {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[0], RAMPMODE_VPOS);
        } else {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[0], RAMPMODE_VNEG);
                velocity.x = -1 * velocity.x;
        }
        if (velocity.y >= 0) {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[1], RAMPMODE_VPOS);
        } else {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[1], RAMPMODE_VNEG);
                velocity.y = -1 * velocity.y;
        }
        
        // Slow motors for debug
        float32_t scale = 0.1;

        // Write motor velocity registers
        uint32_t val;
        val = velocity.x * MAX_VMAX * scale;
        status |= tmc5041_reg_write(tmc5041_VMAX[0], val);
        val = velocity.y * MAX_VMAX * scale;
        status |= tmc5041_reg_write(tmc5041_VMAX[1], val);

        return status;
}

uint8_t pen_stop(void)
{
        struct float32_vec vel;
        vel.x = 0.0f;
        vel.y = 0.0f;
        return = pen_set_velocity(vel);
}
