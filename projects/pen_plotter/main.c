#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

// Included for fast floating point calculations
#include "arm_math.h"

#include "command_usart.h"

/* Preprocessor defines and constants */
// USART command codes
#define COMMAND_RESET 0x01
#define COMMAND_LED   0x02
#define COMMAND_MOVE  0x03

// Default velocity curve values for tmc5041 motor driver
#define MAX_A1         0x000013E8
#define MAX_V1         0x0001c350
#define MAX_AMAX       0x0000FFFF
#define MAX_VMAX       0x001304d0
#define MAX_DMAX       0x000012bc
#define MAX_D1         0x00001578
#define MAX_VSTOP      0x0000000A

#define RAMPMODE_POSITION 0x00000000
#define RAMPMODE_VPOS     0x00000001
#define RAMPMODE_VNEG     0x00000002
#define RAMPMODE_HOLD     0x00000003

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

const uint8_t tmc5041_CHOPCONF[]   = {0x6c, 0x7c};
const uint8_t tmc5041_PWMCONF[]    = {0x10, 0x18};

const uint8_t tmc5041_RAMP_STAT[]  = {0x35, 0x55};

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

// 2D vectors for (x,y) positions and velocities
struct int32_vec {
        int32_t x;
        int32_t y;
};

struct float32_vec {
        float32_t x;
        float32_t y;
};

/* Function prototypes */
// Peripheral Initialisation functions
void clock_init(void);
void io_init(void);
void usart_init(void);
void spi_init(void);
void nvic_init(void);

// GPIO commands - to be moved to a driver
void gpio_set   (uint8_t pin, GPIO_TypeDef* port);
void gpio_clear (uint8_t pin, GPIO_TypeDef* port);
void gpio_toggle(uint8_t pin, GPIO_TypeDef* port);

// TMC5041 motor controller reg read/write functions
void tmc5041_spi_transfer(struct tmc5041_command *command,
                          struct tmc5041_reply   *reply);
void tmc5041_write_reg(uint8_t reg, uint32_t data, uint8_t *status);
uint32_t tmc5041_read_reg (uint8_t reg, uint8_t *status);

// Maths functions
int32_t int_abs(int32_t a);
struct int32_vec vec_add(struct int32_vec A, struct int32_vec B);
struct int32_vec vec_subtract(struct int32_vec A, struct int32_vec B);
int64_t vec_mag_squared(struct int32_vec A);

// Functions for moving pen head
void pen_motors_init(void);
void pen_set_motor_motion_params_max(int motor);
uint8_t pen_set_velocity(struct float32_vec velocity);
uint8_t pen_stop(void);
uint8_t pen_set_target_position(struct int32_vec target);
struct int32_vec pen_get_position(void);
uint32_t pen_motion_check_complete(struct int32_vec start, struct int32_vec end);

// Handler for incoming usart commands
void handle_command(struct command_packet *p);

/* Functions and state variables for motion FIFO */
#define MOTION_FIFO_DEPTH 32
struct int32_vec motion_fifo[MOTION_FIFO_DEPTH];
uint32_t motion_fifo_front = 0;
uint32_t motion_fifo_back  = 0;
uint32_t motion_fifo_full  = 0;
uint32_t motion_fifo_push(struct int32_vec *target);
uint32_t motion_fifo_pop(struct int32_vec *target);
uint32_t motion_fifo_peek(struct int32_vec *target);
uint32_t motion_fifo_check_full(void);
uint32_t motion_fifo_check_empty(void);

int main(void)
{
        clock_init();
        io_init();
        spi_init();
        usart_init();
        nvic_init();
        command_usart_bind(USART1);

        pen_motors_init();

        struct command_packet p;

        GPIOD->ODR = 0x00;

        struct int32_vec start  = pen_get_position();
        struct int32_vec target = start;

        while (1) {
                if (pen_motion_check_complete(start, target)) {
                        if (motion_fifo_check_empty()) {
                                GPIOD->ODR &= ~(1<<13);
                                pen_stop();
                        } else {
                                start = pen_get_position();
                                motion_fifo_pop(&target);
                                pen_set_target_position(target);
                                GPIOD->ODR |= (1<<13);
                        }
                }
                if (command_usart_receive(&p)) {
                        handle_command(&p);
                }
        }
}

struct int32_vec vec_add(struct int32_vec A, struct int32_vec B)
{
        struct int32_vec C;
        C.x = A.x + B.x;
        C.y = A.y + B.y;
        return C;
}

struct int32_vec vec_subtract(struct int32_vec A, struct int32_vec B)
{
        struct int32_vec C;
        C.x = A.x - B.x;
        C.y = A.y - B.y;
        return C;
}

int64_t vec_mag_squared(struct int32_vec A)
{
        int64_t mag_squared;
        mag_squared = (int64_t)A.x*(int64_t)A.x + (int64_t)A.y*(int64_t)A.y;
        return mag_squared;
}

int32_t int_abs(int32_t a)
{
        if (a > 0) {
                return a;
        }
        return -1*a;
}

void handle_command(struct command_packet *p)
{
        int i;
        switch (p->command) {
        case COMMAND_RESET:
                NVIC_SystemReset();
                break;
        case COMMAND_LED:
                if (p->data_length < 4) {
                        break;
                }
                for (i=0; i<4; i++) {
                        if (p->data[i]) {
                                GPIO_SetBits(GPIOD, GPIO_Pin_12 << i);
                        } else {
                                GPIO_ResetBits(GPIOD, GPIO_Pin_12 << i);
                        }
                }
                break;
        case COMMAND_MOVE:
                if (p->data_length < 8) {
                        break;
                }
                struct int32_vec target;
                target.y = (uint32_t)(p->data[7]) << 24 |
                           (uint32_t)(p->data[6]) << 16 |
                           (uint32_t)(p->data[5]) <<  8 |
                           (uint32_t)(p->data[4]) <<  0;
                target.x = (uint32_t)(p->data[3]) << 24 |
                           (uint32_t)(p->data[2]) << 16 |
                           (uint32_t)(p->data[1]) <<  8 |
                           (uint32_t)(p->data[0]) <<  0;
                motion_fifo_push(&target);
                break;
        }
        // Confirm packet was correctly interpreted by returning copy
        command_usart_transmit(p);
}

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
        struct int32_vec distance = vec_subtract(target, position);
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

        struct int32_vec distance_start = vec_subtract(position, start);
        struct int32_vec distance_end   = vec_subtract(position, end);
        struct int32_vec start_end      = vec_subtract(end, start);

        if ((vec_mag_squared(distance_start) > vec_mag_squared(start_end)) |
            (vec_mag_squared(distance_end)   < (int64_t)200000)) {
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
}

void clock_init(void)
{
        // Enable clock for GPIOA
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        // Enable clock for GPIOB
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        // Enable clock for GPIOD (for orange LED)
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);     
        // Enable clock for USART1
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

void io_init(void)
{
        // Connect PB6 to USART1_Tx
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
        // Connect PB7 to USART1_Rx
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

        // Initialization of GPIOB
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIOA->MODER |= (
                GPIO_MODER_MODER3_0 |  // PA3 to generic output (software nSS0)
                GPIO_MODER_MODER4_0 |  // PA4 to generic output (software nSS1)
                GPIO_MODER_MODER5_1 |  // PA5 to alternative function mode (SPI1_CLK)
                GPIO_MODER_MODER6_1 |  // PA6 to alternative function mode (SPI1_MISO)
                GPIO_MODER_MODER7_1    // PA7 to alternative function mode (SPI1_MOSI)
        );

        GPIOA->OSPEEDR |= (
                GPIO_OSPEEDER_OSPEEDR3 | // PA3 output high speed (nSS0)
                GPIO_OSPEEDER_OSPEEDR4 | // PA4 output high speed (nSS1)
                GPIO_OSPEEDER_OSPEEDR5 | // PA5 output high speed (SPI1_CLK)
                GPIO_OSPEEDER_OSPEEDR7   // PA7 output high speed (SPI1_MOSI)
        );

        GPIOA->PUPDR |= (
                GPIO_PUPDR_PUPDR3_0 | // PA3 pull-up (software nSS0)
                GPIO_PUPDR_PUPDR4_0   // PA4 pull-up (software nSS1)
        );

        GPIOA->AFR[0] = 0x55500000; // Set PA5/6/7 to alternative function 5

        // Initialization of GPIOD (for four colour LED)
        GPIO_InitTypeDef GPIO_InitDef;
        GPIO_InitDef.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 |
                                GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
        GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOD, &GPIO_InitDef);
}

void gpio_set(uint8_t pin, GPIO_TypeDef* port)
{
        port->ODR |= (1<<pin);
}

void gpio_clear(uint8_t pin, GPIO_TypeDef* port)
{
        port->ODR &= ~(1<<pin);
}

void gpio_toggle(uint8_t pin, GPIO_TypeDef* port)
{
        port->ODR ^= (1<<pin);
}

void spi_init(void)
{
        // TODO: Need to configure APB2 first
        RCC->APB2ENR |= RCC_APB2RSTR_SPI1RST; // Enable the clock to SPI1
        __asm("dsb");  // stall instruction pipeline, until instruction completes, as
                       // per Errata 2.1.13, "Delay after an RCC peripheral clock enabling"

        // SPI control register 1
        SPI1->CR1 = (
                SPI_CR1_MSTR |     // Set SPI1 to master
                SPI_CR1_CPOL |     // Transfer on rising clock edge
                SPI_CR1_CPHA |     // Capture data on second edge of clock (ignore first falling edge)
                //SPI_CR1_BR_2 |     // TODO: Set correct baud rate. (Set baud rate to f_clk/256)
                SPI_CR1_BR_1 |
                SPI_CR1_BR_0 |
                SPI_CR1_SSM  |     // Software slave management
                SPI_CR1_SSI        // Set the internal SS
        );

        // Enable SPI1
        SPI1->CR1 |= SPI_CR1_SPE;
}

void tmc5041_spi_transfer(struct tmc5041_command *command,
                          struct tmc5041_reply   *reply)
{
        // Dummy read to clear RX buff
        uint8_t dummy = SPI1->DR;

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

void tmc5041_write_reg(uint8_t reg, uint32_t data, uint8_t *status)
{

        struct tmc5041_command command = {
                .reg  = reg + 0x80,
                .data = data
        };
        struct tmc5041_reply dummy;
        tmc5041_spi_transfer(&command, &dummy);
        *status = dummy.status;
}

uint32_t tmc5041_read_reg(uint8_t reg, uint8_t *status)
{
        struct tmc5041_command command = {
                .reg  = reg,
                .data = 0x00000000
        };
        struct tmc5041_reply reply;
        tmc5041_spi_transfer(&command, &reply);
        tmc5041_spi_transfer(&command, &reply);

        *status = reply.status;
        return reply.data;
}

void usart_init(void)
{
        // Initialization of USART1
        USART_InitTypeDef USART_InitStruct;
        USART_InitStruct.USART_BaudRate = 115200;
        USART_InitStruct.USART_HardwareFlowControl =
                USART_HardwareFlowControl_None;
        USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_InitStruct.USART_Parity = USART_Parity_No;
        USART_InitStruct.USART_StopBits = USART_StopBits_1;
        USART_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART_Init(USART1, &USART_InitStruct);

        // Enable USART1
        USART_Cmd(USART1, ENABLE);

        // Enable RX interrupt on USART1
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void nvic_init(void) {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}


uint32_t motion_fifo_push(struct int32_vec *target)
{
        if (motion_fifo_check_full()) {
                return 0;
        }
        motion_fifo[motion_fifo_back].x = target->x;
        motion_fifo[motion_fifo_back].y = target->y;
        motion_fifo_back++;

        if (motion_fifo_back == motion_fifo_front) {
                motion_fifo_full = 1;
        }
        return 1;
}

uint32_t motion_fifo_pop(struct int32_vec *target)
{
        if (motion_fifo_check_empty()) {
                return 0;
        }
        target->x = motion_fifo[motion_fifo_front].x;
        target->y = motion_fifo[motion_fifo_front].y;
        motion_fifo_front++;
        motion_fifo_full = 0;
        return 1;
}

uint32_t motion_fifo_peek(struct int32_vec *target)
{
        if (motion_fifo_check_empty()) {
                return 0;
        }
        target->x = motion_fifo[motion_fifo_front].x;
        target->y = motion_fifo[motion_fifo_front].y;
        return 1;
}

uint32_t motion_fifo_check_full(void)
{
        return motion_fifo_full;
}

uint32_t motion_fifo_check_empty(void)
{
        return (!motion_fifo_full) && (motion_fifo_front == motion_fifo_back);
}
