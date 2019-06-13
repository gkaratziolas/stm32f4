#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#include "arm_math.h"

#include "debug_usart.h"

#define MAX_A1         0x000013E8
#define MAX_V1         0x0001c350
#define MAX_AMAX       0x000011f4
#define MAX_VMAX       0x001304d0
#define MAX_DMAX       0x000012bc
#define MAX_D1         0x00001578
#define MAX_VSTOP      0x0000000A
#define MAX_RAMPMODE   0x00000000

const uint8_t tmc5041_GCONF = 0x00;
const uint8_t tmc5041_GSTAT = 0x01;

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


struct tmc5041_command {
        uint8_t  reg;
        uint32_t data;
};

struct tmc5041_reply {
        uint8_t  status;
        uint32_t data;
};


void clock_init(void);
void io_init(void);
void usart_init(void);
void spi_init(void);

void gpio_set   (uint8_t pin, GPIO_TypeDef* port);
void gpio_clear (uint8_t pin, GPIO_TypeDef* port);
void gpio_toggle(uint8_t pin, GPIO_TypeDef* port);

void     tmc5041_write_reg(uint8_t reg, uint32_t data, uint8_t *status);
uint32_t tmc5041_read_reg (uint8_t reg, uint8_t *status);

int32_t int_abs(int32_t a);

void pen_motors_init(void);
void pen_set_motor_max_speed(int motor);
void pen_set_motor_rel_speed(int motor, float32_t scale);
uint8_t pen_goto_motor_rotation(int32_t A, int32_t B);

void flower_demo();
void circle_demo();

int main(void)
{
        clock_init();
        io_init();
        spi_init();
        usart_init();
        debug_usart_bind(USART1);

        pen_motors_init();

        // Send "Hello, World!" to PC
        debug_usart_print("Hello, World!\n");

        GPIOD->MODER = (1 << 24) | (1 << 26) | (1 << 28) | (1 << 30); // set pin 13 to be general purpose output

        uint8_t status;

        uint32_t e = 0;
        while(1) {
                flower_demo();

                e += 10000;
                debug_usart_print("aah!\n");
                // Toggle LEDs
                GPIOD->ODR ^=  (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);
        }
}

void flower_demo()
{
        int i = 0;
        int i_MAX = 18;
        
        float32_t sin_val, cos_val;
        float32_t theta = 0;
        // theta
        float32_t theta_step = 0.349066; // 2pi/18
        float32_t amplitude  = 50000;
        float32_t A, B;
        int32_t rotA, rotB;

        for (i=0; i<i_MAX; i++) {
                sin_val = arm_sin_f32(theta);
                cos_val = arm_cos_f32(theta);

                arm_mult_f32(&sin_val, &amplitude, &A, 1);
                arm_mult_f32(&cos_val, &amplitude, &B, 1);

                rotA = (int32_t) A;
                rotA = (int32_t) B;

                pen_goto_motor_rotation(A, B);
                pen_goto_motor_rotation(B, -1*A);
                pen_goto_motor_rotation(-1*A, -1*B);
                pen_goto_motor_rotation(-1*B, A);
                pen_goto_motor_rotation(A, B);

                arm_add_f32(&theta, &theta_step, &theta, 1);
        }
}

void circle_demo()
{
        int i = 0;
        int i_MAX = 18;
        
        float32_t sin_val, cos_val;
        float32_t theta = 0;
        // theta
        float32_t theta_step = 0.349066; // 2pi/18
        float32_t amplitude  = 50000;
        float32_t A, B;
        int32_t rotA, rotB;

        for (i=0; i<i_MAX; i++) {
                sin_val = arm_sin_f32(theta);
                cos_val = arm_cos_f32(theta);

                arm_mult_f32(&sin_val, &amplitude, &A, 1);
                arm_mult_f32(&cos_val, &amplitude, &B, 1);

                rotA = (int32_t) A;
                rotA = (int32_t) B;

                pen_goto_motor_rotation(A, B);

                arm_add_f32(&theta, &theta_step, &theta, 1);
        }
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
                tmc5041_write_reg(tmc5041_TZEROWAIT[i],  0x00002710, &status);
                tmc5041_write_reg(tmc5041_PWMCONF[i],    0x000401c8, &status);
                tmc5041_write_reg(tmc5041_VHIGH[i],      0x00061a80, &status);
                tmc5041_write_reg(tmc5041_VCOOLTHRS[i],  0x00007530, &status); 
        }

        // Motor motion config
        for (i=0; i<2; i++) {
                pen_set_motor_max_speed(i);
        }
}

void pen_set_motor_max_speed(int motor)
{
        uint8_t status;
        tmc5041_write_reg(tmc5041_A1[motor],       MAX_A1,       &status);
        tmc5041_write_reg(tmc5041_V1[motor],       MAX_V1,       &status);
        tmc5041_write_reg(tmc5041_AMAX[motor],     MAX_AMAX,     &status);
        tmc5041_write_reg(tmc5041_VMAX[motor],     MAX_VMAX,     &status);
        tmc5041_write_reg(tmc5041_DMAX[motor],     MAX_DMAX,     &status);
        tmc5041_write_reg(tmc5041_D1[motor],       MAX_D1,       &status);
        tmc5041_write_reg(tmc5041_VSTOP[motor],    MAX_VSTOP,    &status);
        tmc5041_write_reg(tmc5041_RAMPMODE[motor], MAX_RAMPMODE, &status);      
}

void pen_set_motor_rel_speed(int motor, float32_t scale)
{
        int32_t val;
        uint8_t status;

        if (scale > 1) {
                scale = 1;
        }

        val = scale * MAX_A1;
        tmc5041_write_reg(tmc5041_A1[motor],    val, &status);
        val = scale * MAX_V1;
        tmc5041_write_reg(tmc5041_V1[motor],    val, &status);
        val = scale * MAX_AMAX;
        tmc5041_write_reg(tmc5041_AMAX[motor],  val, &status);
        val = scale * MAX_VMAX;
        tmc5041_write_reg(tmc5041_VMAX[motor],  val, &status);
        val = scale * MAX_DMAX;
        tmc5041_write_reg(tmc5041_DMAX[motor],  val, &status);
        val = scale * MAX_D1;
        tmc5041_write_reg(tmc5041_D1[motor],    val, &status);
        val = scale * MAX_VSTOP;
        tmc5041_write_reg(tmc5041_VSTOP[motor], val, &status);
}

int32_t int_abs(int32_t a)
{
        if (a > 0) {
                return a;
        }
        return -1*a;
}

uint8_t pen_goto_motor_rotation(int32_t A, int32_t B)
{
        uint8_t status;

        int32_t A0 = tmc5041_read_reg(tmc5041_XACTUAL[0], &status);
        int32_t B0 = tmc5041_read_reg(tmc5041_XACTUAL[1], &status);

        int32_t distA = int_abs(A - A0);
        int32_t distB = int_abs(B - B0);

        if (distA >= distB) {
                pen_set_motor_max_speed(0);
                pen_set_motor_rel_speed(1, (float32_t)distB/distA);
        } else {
                pen_set_motor_max_speed(1);
                pen_set_motor_rel_speed(0, (float32_t)distA/distB);
        }

        tmc5041_write_reg(tmc5041_XTARGET[0], A, &status);
        tmc5041_write_reg(tmc5041_XTARGET[1], B, &status);

        // Wait until location is reached
        // TODO: Add timeout
        //while (!(tmc5041_read_reg(tmc5041_RAMP_STAT[0], &status) & (1<<9))) {__asm("nop");}
        //while (!(tmc5041_read_reg(tmc5041_RAMP_STAT[1], &status) & (1<<9))) {__asm("nop");}

        for (int i=0; i<50000000; i++) {__asm("nop");}

        return status;
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
                          struct tmc5041_reply   *reply
                          )
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
}
