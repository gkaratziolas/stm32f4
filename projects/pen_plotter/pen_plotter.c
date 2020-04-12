#include "pen_plotter.h"

#include "tmc5041.h"
// TODO: #include "servo_generic.h"

void pen_motors_init()
{
        int i;
        uint8_t status;
        tmc5041_write_reg(tmc5041_GCONF, 0x00000008, &status);

        // Initial motor config
        for (i=0; i<2; i++) {
                tmc5041_write_reg(tmc5041_CHOPCONF[i],   0x000100c5, &status);
                tmc5041_write_reg(tmc5041_IHOLD_IRUN[i], 0x00011f05, &status);
                tmc5041_write_reg(tmc5041_TZEROWAIT[i],  0x00000000, &status);
                tmc5041_write_reg(tmc5041_PWMCONF[i],    0x000401c8, &status);
                tmc5041_write_reg(tmc5041_VHIGH[i],      0x00061a80, &status);
                tmc5041_write_reg(tmc5041_VCOOLTHRS[i],  0x00007530, &status);

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
                pen_set_motor_motion_params_max(i);
        }
}

void pen_set_motor_motion_params_max(int motor)
{
        uint8_t status;
        tmc5041_write_reg(tmc5041_A1[motor],       MAX_A1,            &status);
        tmc5041_write_reg(tmc5041_V1[motor],       MAX_V1,            &status);
        tmc5041_write_reg(tmc5041_AMAX[motor],     MAX_AMAX,          &status);
        tmc5041_write_reg(tmc5041_VMAX[motor],     MAX_VMAX,          &status);
        tmc5041_write_reg(tmc5041_DMAX[motor],     MAX_DMAX,          &status);
        tmc5041_write_reg(tmc5041_D1[motor],       MAX_D1,            &status);
        tmc5041_write_reg(tmc5041_VSTOP[motor],    MAX_VSTOP,         &status);
        tmc5041_write_reg(tmc5041_RAMPMODE[motor], RAMPMODE_POSITION, &status);      
}

struct int32_vec pen_get_position(void)
{
        struct int32_vec pos;
        uint8_t status;
        pos.x = tmc5041_read_reg(tmc5041_XACTUAL[0], &status);
        pos.y = tmc5041_read_reg(tmc5041_XACTUAL[1], &status);
        return pos;
}

uint8_t pen_set_target_position(struct int32_vec target)
{
        struct int32_vec position = pen_get_position();
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
        
        uint8_t status = pen_set_velocity(velocity);
        return status;
}

uint32_t pen_motion_check_complete(struct int32_vec start, struct int32_vec end)
{
        struct int32_vec position = pen_get_position();

        struct int32_vec distance_start = int32_vec_sub(&position, &start);
        struct int32_vec distance_end   = int32_vec_sub(&position, &end);
        struct int32_vec start_end      = int32_vec_sub(&end, &start);

        if ((int32_vec_mag_squared(&distance_start) > int32_vec_mag_squared(&start_end)) |
            (int32_vec_mag_squared(&distance_end)   < (int64_t)200000)) {
                return 1;
        }
        return 0;
}

// Set velocity yo some fraction of v_max between +/-1
uint8_t pen_set_velocity(struct float32_vec velocity)
{
        uint8_t status;

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
                tmc5041_write_reg(tmc5041_RAMPMODE[0], RAMPMODE_VPOS, &status);
        } else {
                tmc5041_write_reg(tmc5041_RAMPMODE[0], RAMPMODE_VNEG, &status);
                velocity.x = -1 * velocity.x;
        }
        if (velocity.y >= 0) {
                tmc5041_write_reg(tmc5041_RAMPMODE[1], RAMPMODE_VPOS, &status);
        } else {
                tmc5041_write_reg(tmc5041_RAMPMODE[1], RAMPMODE_VNEG, &status);
                velocity.y = -1 * velocity.y;
        }
        
        // Slow motors for debug
        float32_t scale = 0.1;

        // Write motor velocity registers
        uint32_t val;
        val = velocity.x * MAX_VMAX * scale;
        tmc5041_write_reg(tmc5041_VMAX[0], val, &status);
        val = velocity.y * MAX_VMAX * scale;
        tmc5041_write_reg(tmc5041_VMAX[1], val, &status);

        return status;
}

uint8_t pen_stop(void)
{
        struct float32_vec speed;
        speed.x = 0.0f;
        speed.y = 0.0f;
        uint8_t status = pen_set_velocity(speed);
        return status;
}
