#include "pen_plotter.h"

#include "math_utils.h"
#include "tmc5041.h"
#include "gcode_reader.h"

// TODO: #include "servo_generic.h"

#ifdef DEBUG
#include "debug_usart.h"
#endif //DEBUG_

/*
 ****************************************
 * Private types for describing motions *
 ****************************************
 */

enum motion_type {
        MOTION_NONE,
        MOTION_AB,
        MOTION_Z,
};

struct motion {
        int type;
        union {
                struct int32_vec ab;
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
uint8_t pen_motion_start_AB(struct int32_vec target);
void pen_motion_start_Z(uint8_t target);

/* Check for motion complete */
int pen_motion_check_complete(struct motion *target);
int pen_motion_check_complete_AB(struct int32_vec *target);
int pen_motion_check_complete_Z(uint8_t *target);

/* tmc5041 interface functions */
uint8_t pen_motors_init(void);
uint8_t pen_motor_set_motion_params_max(int motor);
uint8_t pen_get_position(struct int32_vec *position);
uint8_t pen_set_velocity(struct float32_vec velocity);
uint8_t pen_stop(void);

/*
 ************************************
 * Private gcode handling functions *
 ************************************
 */

/* Generic handeler, will redirect command to specific handler */
int gcode_decode(struct gcode_command *gcommand);

/* Decode Gxx codes - perform motions */
int gcode_decode_G00(struct gcode_command *gcommand);
int gcode_decode_G01(struct gcode_command *gcommand);
int gcode_decode_G02(struct gcode_command *gcommand);
int gcode_decode_G03(struct gcode_command *gcommand);
int gcode_decode_G21(struct gcode_command *gcommand);

/* Decode Mxx codes - set motion parameters */
int gcode_decode_M02(struct gcode_command *gcommand);
int gcode_decode_M03(struct gcode_command *gcommand);
int gcode_decode_M05(struct gcode_command *gcommand);


/* Servo interface functions */
// TODO!

/*
 **************************************
 * Static global variable definitions *
 **************************************
 */

/* Motion FIFO for storing goto ab / goto z commands */
struct motion motions[MAX_MOTIONS];
struct fifo motion_fifo;

/*
 * Starting point for AB motions.
 * Referenced when checking motion complete.
 * Updated once AB motion is finished
 */
static struct int32_vec motion_AB_source;

/*
 * Starting point for xy gcode decoding.
 * Updated whenever a gcode command is decoded.
 * Used as reference starting point for any circular motions.
 */
static struct float32_vec latest_xy_target;

struct fifo *gcommand_fifo = NULL;

static uint32_t pen_mm_conv_factor = 500;

/*
 ************************
 * Function definitions *
 ************************
 */

void pen_init(struct fifo *g)
{
        motion_fifo = fifo_init(motions, sizeof(struct motion), MAX_MOTIONS);
        gcommand_fifo = g;

        // TODO: Should be updated by homing routine
        latest_xy_target.x = 0;
        latest_xy_target.y = 0;
        motion_AB_source.a = 0;
        motion_AB_source.b = 0;

        pen_motors_init();
}

void pen_deinit(void)
{
        pen_stop();
        gcommand_fifo = (struct fifo *)NULL;
}

void pen_serve()
{
        struct gcode_command gcommand;
        if ((!fifo_empty(gcommand_fifo)) &&
             (fifo_space(&motion_fifo) >= MAX_MOTIONS_PER_GCOMMAND)) {
                fifo_pop(gcommand_fifo, &gcommand);
                gcode_decode(&gcommand);
        }

        static struct motion target = {.type = MOTION_NONE}; 
        if (!pen_motion_check_complete(&target)) {
                return;
        }
        if (fifo_pop(&motion_fifo, &target) == FIFO_ERR_EMPTY) {
                target.type = MOTION_NONE;
        }
        pen_motion_start(&target);
}

int pen_motion_check_complete(struct motion *target)
{
        switch(target->type) {
        case MOTION_AB:
                if (pen_motion_check_complete_AB(&(target->ab))) {
                        return 1;
                }
                return 0;
        case MOTION_Z:
                if (pen_motion_check_complete_Z(&(target->z))) {
                        return 1;
                }
                return 0;
        case MOTION_NONE:
                return 1;
        default:
                return 1; 
        }
}

int pen_motion_check_complete_AB(struct int32_vec *target)
{
        struct int32_vec position;
        pen_get_position(&position);

        struct int32_vec dist_to_start  = int32_vec_sub(&position, &motion_AB_source);
        struct int32_vec dist_to_end    = int32_vec_sub(&position, target);
        struct int32_vec dist_start_end = int32_vec_sub(target, &motion_AB_source);

        // If the pen is close to the target, or has travelled further than the
        // target from the start, then the motion is complete
        if ((int32_vec_mag_squared(&dist_to_start) > int32_vec_mag_squared(&dist_start_end)) |
            (int32_vec_mag_squared(&dist_to_end)   < DIST_SQUARED_TARGET_THRESHOLD)) {
                // update source position for future AB motions
                motion_AB_source.a = position.a;
                motion_AB_source.b = position.b;
                return 1;
        }
        return 0;
}

int pen_motion_check_complete_Z(uint8_t *target)
{
        // TODO: check servo position here
        return 1;
}

void pen_motion_start(struct motion *target)
{
        switch(target->type) {
        case MOTION_AB:
                pen_motion_start_AB(target->ab);
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

uint8_t pen_motion_start_AB(struct int32_vec target)
{
        struct int32_vec position;
        pen_get_position(&position);
        struct int32_vec distance = int32_vec_sub(&target, &position);
        struct float32_vec velocity;

        if (int32_abs(distance.a) >= int32_abs(distance.b)) {
                if (distance.a > 0)
                        velocity.a = 1.0f;
                else
                        velocity.a = -1.0f;
                velocity.b = ((float32_t)distance.b * velocity.a) / (float32_t)distance.a;
        } else {
                if (distance.b > 0)
                        velocity.b = 1.0f;
                else
                        velocity.b = -1.0f;
                velocity.a = ((float32_t)distance.a * velocity.b) / (float32_t)distance.b;
        }
        
        motion_AB_source = position;
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
                status |= pen_motor_set_motion_params_max(i);
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
        if (velocity.a > 1.0f)
                velocity.a = 1.0f;
        if (velocity.a < -1.0f)
                velocity.a = -1.0f;
        if (velocity.b > 1)
                velocity.b = 1;
        if (velocity.b < -1)
                velocity.b = -1;

        // Check whether velocity components are positive or negative
        if (velocity.a >= 0) {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[0], RAMPMODE_VPOS);
        } else {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[0], RAMPMODE_VNEG);
                velocity.a = -1 * velocity.a;
        }
        if (velocity.b >= 0) {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[1], RAMPMODE_VPOS);
        } else {
                status |= tmc5041_reg_write(tmc5041_RAMPMODE[1], RAMPMODE_VNEG);
                velocity.b = -1 * velocity.b;
        }
        
        // Slow motors for debug
        float32_t scale = 0.1;

        // Write motor velocity registers
        uint32_t val;
        val = velocity.a * MAX_VMAX * scale;
        status |= tmc5041_reg_write(tmc5041_VMAX[0], val);
        val = velocity.b * MAX_VMAX * scale;
        status |= tmc5041_reg_write(tmc5041_VMAX[1], val);

        return status;
}

uint8_t pen_stop(void)
{
        struct float32_vec vel;
        vel.a = 0.0f;
        vel.b = 0.0f;
        return pen_set_velocity(vel);
}

void convert_z_mm_to_pen_Z(float32_t *z_mm, uint8_t *z)
{
        if (z_mm > 0) {
                *z = 255;
        } else {
                *z = 0;
        }
}

void convert_xy_mm_to_AB(struct float32_vec *xy_mm, struct int32_vec *ab)
{
        int32_t x = (int32_t) (xy_mm->x * pen_mm_conv_factor);
        int32_t y = (int32_t) (xy_mm->y * pen_mm_conv_factor);
        ab->a = x - y;
        ab->b = x + y;
}

int gcode_decode(struct gcode_command *gcommand)
{
        int status;
        switch (gcommand->code) {
        case gcode_G00:
                status = gcode_decode_G00(gcommand);
                break;
        case gcode_G01:
                status = gcode_decode_G01(gcommand);
                break;
        case gcode_G02:
        case gcode_G03:
                status = gcode_decode_G02_G03(gcommand);
                break;
        case gcode_G21:
                status = gcode_decode_G21(gcommand);
                break;
        case gcode_M02:
                status = gcode_decode_M02(gcommand);
                break;
        case gcode_M03:
                status = gcode_decode_M03(gcommand);
                break;
        case gcode_M05:
                status = gcode_decode_M05(gcommand);
                break;
        case gcode_GXX:
        case gcode_MXX:
        case gcode_NONE:
        default:        
                status = 0;
                break;
        }
        return status;
}

int gcode_decode_G00(struct gcode_command *gcommand)
{
        float z_mm;
        struct float32_vec xy_mm;
        struct motion target;

        if (gcode_command_read_var(gcommand, 'Z', &z_mm) == GCODE_VAR_FOUND) {
                target.type = MOTION_Z;
                convert_z_mm_to_pen_Z(&z_mm, &(target.z));
                fifo_push(&motion_fifo, &target);
        }
        if ((gcode_command_read_var(gcommand, 'X', &(xy_mm.x)) == GCODE_VAR_FOUND) &&
            (gcode_command_read_var(gcommand, 'Y', &(xy_mm.y)) == GCODE_VAR_FOUND)) {
                target.type = MOTION_AB;
                convert_xy_mm_to_AB(&xy_mm, &(target.ab));
                fifo_push(&motion_fifo, &target);
                latest_xy_target.x = xy_mm.x;
                latest_xy_target.y = xy_mm.y;
        }

        return 0;
}

int gcode_decode_G01(struct gcode_command *gcommand)
{
        return gcode_decode_G00(gcommand);
}

int gcode_decode_G02_G03(struct gcode_command *gcommand)
{
        // Read out parameters
        // TODO: Add support for X, Y, R parameter set
        struct float32_vec XY_mm;
        struct float32_vec IJ_mm;
        if ((gcode_command_read_var(gcommand, 'X', &(XY_mm.x)) == GCODE_VAR_NOT_FOUND) ||
            (gcode_command_read_var(gcommand, 'Y', &(XY_mm.y)) == GCODE_VAR_NOT_FOUND) ||
            (gcode_command_read_var(gcommand, 'I', &(IJ_mm.x)) == GCODE_VAR_NOT_FOUND) ||
            (gcode_command_read_var(gcommand, 'J', &(IJ_mm.y)) == GCODE_VAR_NOT_FOUND)) {
                return -1;
        }

        // Calculate start and end position vectors
        struct float32_vec centre = float32_vec_add(&latest_xy_target, &IJ_mm);
        struct float32_vec start  = float32_vec_sub(&latest_xy_target, &centre);
        struct float32_vec target = float32_vec_sub(&XY_mm, &centre);

        // Calculate angle between start and end point (relative to centre)
        float32_t theta;
        if (gcommand->code == gcode_G02) {
                theta = cw_angle(&start, &target);
        } else if (gcommand->code == gcode_G03) {
                theta = ccw_angle(&start, &target);
        } else {
                // error
                return -1;
        }

        // Calculate arc length and number of points in arc approximation
        float32_t arc_length = theta * float32_vec_mag(&start);
        int32_t N = (int32_t) (arc_length / ARC_SEGMENT_LENGTH);
        if (N > MAX_MOTIONS_PER_GCOMMAND)
                N = MAX_MOTIONS_PER_GCOMMAND;
        float32_t d_theta = theta / N;

        char buf[255];
        sprintf(buf, "%f, %d\n", theta, N);
        debug_usart_print(buf);

        /* Prepare d_theta rotation matrix
         *
         *        (xx yx)
         *        (xy yy)
         */
        float32_t rot_xx;
        float32_t rot_yx;
        float32_t rot_xy;
        float32_t rot_yy;
        float32_t sin_d_theta = arm_sin_f32(d_theta);
        float32_t cos_d_theta = arm_cos_f32(d_theta);
        if (gcommand->code == gcode_G02) {
                rot_xx =  cos_d_theta;
                rot_xy = -1 * sin_d_theta;
                rot_yx =      sin_d_theta;
                rot_yy =  cos_d_theta;
        } else if (gcommand->code == gcode_G03) {
                rot_xx =  cos_d_theta;
                rot_xy =       sin_d_theta;
                rot_yx =  -1 * sin_d_theta;
                rot_yy =  cos_d_theta;
        } else {
                // error
                return -1;
        }

        // Successively apply d_theta rotation matrix until at target
        struct float32_vec pos;
        struct float32_vec pos_copy;
        struct float32_vec pos_abs;
        struct motion ntarget;
        int32_t i;
        pos.x = start.x;
        pos.y = start.y;
        for (i=0; i<N; i++) {
                // Copy position
                pos_copy.x = pos.x;
                pos_copy.y = pos.y;

                // Apply rotation matrix
                pos.x = (pos_copy.x * rot_xx) + (pos_copy.y * rot_yx);
                pos.y = (pos_copy.x * rot_xy) + (pos_copy.y * rot_yy);
                
                // Add offset vector to position;
                pos_abs.x = centre.x + pos.x;
                pos_abs.y = centre.y + pos.y;

                // Push new target to motion fifo
                ntarget.type = MOTION_AB;
                convert_xy_mm_to_AB(&pos_abs, &(ntarget.ab));
                fifo_push(&motion_fifo, &ntarget);
        }

        // Update latest target for use by the next G02/G03 command
        latest_xy_target.x = XY_mm.x;
        latest_xy_target.y = XY_mm.y;
        return 0;
}

int gcode_decode_G21(struct gcode_command *gcommand)
{
        return 0;
}

int gcode_decode_M02(struct gcode_command *gcommand)
{
        return 0;
}

int gcode_decode_M03(struct gcode_command *gcommand)
{
        return 0;
}

int gcode_decode_M05(struct gcode_command *gcommand)
{
        return 0;
}
