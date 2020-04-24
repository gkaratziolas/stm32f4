#ifndef TMC5041_H_
#define TMC5041_H_

#include "stm32f4xx.h"
#include "math_utils.h"

// Status byte defines
#define STATUS_BIT_RESET_FLAG         0
#define STATUS_BIT_DRIVER_ERROR_1     1
#define STATUS_BIT_DRIVER_ERROR_2     2
#define STATUS_BIT_VELOCITY_REACHED_1 3
#define STATUS_BIT_VELOCITY_REACHED_2 4
#define STATUS_BIT_STATUS_STOP_1      5
#define STATUS_BIT_STATUS_STOP_1      6
#define STATUS_BIT_RESERVED           7

#define STATUS_RESET_FLAG            (uint8_t)(0x01 << STATUS_BIT_RESET_FLAG)
#define STATUS_DRIVER_ERROR_1        (uint8_t)(0x01 << STATUS_BIT_DRIVER_ERROR_1)
#define STATUS_DRIVER_ERROR_2        (uint8_t)(0x01 << STATUS_BIT_DRIVER_ERROR_2)
#define STATUS_VELOCITY_REACHED_1    (uint8_t)(0x01 << STATUS_BIT_VELOCITY_REACHED_1)
#define STATUS_VELOCITY_REACHED_2    (uint8_t)(0x01 << STATUS_BIT_VELOCITY_REACHED_2)
#define STATUS_STATUS_STOP_1         (uint8_t)(0x01 << STATUS_BIT_STATUS_STOP_1)
#define STATUS_STATUS_STOP_1         (uint8_t)(0x01 << STATUS_BIT_STATUS_STOP_1)
#define STATUS_RESERVED              (uint8_t)(0x01 << STATUS_BIT_RESERVED)

// TMC5041 register addresses
extern const uint8_t tmc5041_GCONF;
extern const uint8_t tmc5041_GSTAT;
extern const uint8_t tmc5041_RAMPMODE[];
extern const uint8_t tmc5041_XACTUAL[];
extern const uint8_t tmc5041_A1[];
extern const uint8_t tmc5041_V1[];
extern const uint8_t tmc5041_D1[];
extern const uint8_t tmc5041_AMAX[];
extern const uint8_t tmc5041_VMAX[];
extern const uint8_t tmc5041_DMAX[];
extern const uint8_t tmc5041_VSTOP[];
extern const uint8_t tmc5041_TZEROWAIT[];
extern const uint8_t tmc5041_XTARGET[];
extern const uint8_t tmc5041_IHOLD_IRUN[];
extern const uint8_t tmc5041_VCOOLTHRS[];
extern const uint8_t tmc5041_VHIGH[];
extern const uint8_t tmc5041_SW_MODE[];
extern const uint8_t tmc5041_PWMCONF[];
extern const uint8_t tmc5041_CHOPCONF[];
extern const uint8_t tmc5041_COOLCONF[];
extern const uint8_t tmc5041_RAMP_STAT[];
extern const uint8_t tmc5041_DRV_STATUS[];

// Register read/write
uint8_t tmc5041_reg_write(uint8_t reg, uint32_t data);
uint8_t tmc5041_reg_read(uint8_t reg, uint32_t *data);

// Alter register content
uint8_t tmc5041_reg_mask(uint8_t reg, uint32_t mask);
uint8_t tmc5041_reg_bit_set(uint8_t reg, uint8_t bit);
uint8_t tmc5041_reg_bit_reset(uint8_t reg, uint8_t bit);


#endif // TMC5041_H_
