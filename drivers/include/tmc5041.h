#ifndef TMC5041_H_
#define TMC5041_H_

#include "stm32f4xx.h"
#include "math_utils.h"

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

/* Structs */
// TMC5041 command structs
struct tmc5041_command {
        uint8_t  reg;
        uint32_t data;
};

struct tmc5041_reply {
        uint8_t  status;
        uint32_t data;
};

// TMC5041 motor controller reg read/write functions
void tmc5041_spi_transfer(struct tmc5041_command *command,
                          struct tmc5041_reply   *reply);
void tmc5041_write_reg(uint8_t reg, uint32_t data, uint8_t *status);
uint32_t tmc5041_read_reg (uint8_t reg, uint8_t *status);
uint32_t tmc5041_mask_reg(uint8_t reg, uint32_t mask, uint8_t *status);
void tmc5041_set_reg_bit(uint8_t reg, uint8_t bit, uint8_t *status);
void tmc5041_reset_reg_bit(uint8_t reg, uint8_t bit, uint8_t *status);

#endif // TMC5041_H_
