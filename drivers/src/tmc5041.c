#include "tmc5041.h"

#include "stm32f4xx_i2c.h"

#include "gpio.h"

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

// TMC5041 register addresses
const uint8_t tmc5041_GCONF        = 0x00;
const uint8_t tmc5041_GSTAT        = 0x01;

const uint8_t tmc5041_RAMPMODE[]   = {0x20, 0x40};
const uint8_t tmc5041_XACTUAL[]    = {0x21, 0x41};

const uint8_t tmc5041_A1[]         = {0x24, 0x44};
const uint8_t tmc5041_V1[]         = {0x25, 0x45};
const uint8_t tmc5041_D1[]         = {0x2a, 0x4a};
const uint8_t tmc5041_AMAX[]       = {0x26, 0x46};
const uint8_t tmc5041_VMAX[]       = {0x27, 0x47};
const uint8_t tmc5041_DMAX[]       = {0x28, 0x48};

const uint8_t tmc5041_VSTOP[]      = {0x2b, 0x4b};
const uint8_t tmc5041_TZEROWAIT[]  = {0x2c, 0x4c};
const uint8_t tmc5041_XTARGET[]    = {0x2d, 0x4d};

const uint8_t tmc5041_IHOLD_IRUN[] = {0x30, 0x50};
const uint8_t tmc5041_VCOOLTHRS[]  = {0x31, 0x51};

const uint8_t tmc5041_VHIGH[]      = {0x32, 0x52};

const uint8_t tmc5041_SW_MODE[]    = {0x34, 0x54};

const uint8_t tmc5041_PWMCONF[]    = {0x10, 0x18};
const uint8_t tmc5041_CHOPCONF[]   = {0x6c, 0x7c};
const uint8_t tmc5041_COOLCONF[]   = {0x6d, 0x7d};

const uint8_t tmc5041_RAMP_STAT[]  = {0x35, 0x55};
const uint8_t tmc5041_DRV_STATUS[] = {0x6F, 0x7F};

void tmc5041_spi_transfer(struct tmc5041_command *command,
                          struct tmc5041_reply   *reply)
{
        // Dummy read to clear RX buff
        (void)(SPI1->DR);

        // Set nSS low (SPI active)
        gpio_clear(4, GPIOA);

        // Wait for SPI1 to become available
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}

        // Transfer first bit
        SPI1->DR = command->reg;
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        // Wait for RX data and read
        while(!(SPI1->SR & SPI_SR_RXNE)) {__asm("nop");}
        reply->status = SPI1->DR;

        // Exchange data[31:24]
        SPI1->DR = ((command->data & 0xff000000) >> 24);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        // Wait for RX data and read
        while(!(SPI1->SR & SPI_SR_RXNE)) {__asm("nop");}
        reply->data = ((uint32_t)SPI1->DR) << 24;

        // Exchange data[23:16]
        SPI1->DR = ((command->data & 0x00ff0000) >> 16);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        // Wait for RX data and read
        while(!(SPI1->SR & SPI_SR_RXNE)) {__asm("nop");}
        reply->data |= ((uint32_t)SPI1->DR) << 16;

        // Exchange data[15:8]
        SPI1->DR = ((command->data & 0x0000ff00) >> 8);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        // Wait for RX data and read
        while(!(SPI1->SR & SPI_SR_RXNE)) {__asm("nop");}
        reply->data |= ((uint32_t)SPI1->DR) << 8;

        // Exchange data[7:0]
        SPI1->DR = ((command->data & 0x000000ff) >> 0);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        // Wait for RX data and read
        while(!(SPI1->SR & SPI_SR_RXNE)) {__asm("nop");}
        reply->status |= (uint32_t)SPI1->DR;

        // Set nSS high (SPI inactive)
        gpio_set(4, GPIOA);
}

uint8_t tmc5041_reg_write(uint8_t reg, uint32_t data)
{

        struct tmc5041_command command = {
                .reg  = reg + 0x80,
                .data = data
        };
        struct tmc5041_reply dummy;
        tmc5041_spi_transfer(&command, &dummy);
        return dummy.status;
}

uint8_t tmc5041_reg_read(uint8_t reg, uint32_t *data)
{
        struct tmc5041_command command = {
                .reg  = reg,
                .data = 0x00000000
        };
        struct tmc5041_reply reply;
        tmc5041_spi_transfer(&command, &reply);
        tmc5041_spi_transfer(&command, &reply);

        *data = reply.data;

        return reply.status;
}

uint8_t tmc5041_reg_mask(uint8_t reg, uint32_t mask)
{
        uint32_t data;
        uint8_t status = tmc5041_read_reg(reg, &data);
        data &= mask;
        tmc5041_write_reg(reg, data);
        return status;
}

uint8_t tmc5041_reg_bit_set(uint8_t reg, uint8_t bit)
{
        uint32_t data;
        uint8_t status = tmc5041_read_reg(reg, &data);
        data |= (1 << bit);
        status |= tmc5041_write_reg(reg, data);
        return status;
}

uint8_t tmc5041_reg_bit_reset(uint8_t reg, uint8_t bit)
{
        uint32_t data;
        uint8_t status = tmc5041_read_reg(reg, &data);
        data &= ~(1 << bit);
        status |= tmc5041_write_reg(reg, data);
        return status;      
}
