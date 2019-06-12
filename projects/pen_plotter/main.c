#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#include "debug_usart.h"

const uint8_t tmc5041_GCONF = 0x00;
const uint8_t tmc5041_GSTAT = 0x01;

const uint8_t tmc5041_RAMPMODE[]   = {0x20, 0x40};

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
uint32_t tmc5041_read_reg (uint8_t reg, uint32_t data, uint8_t *status);


int main(void)
{
        clock_init();
        io_init();
        spi_init();
        usart_init();
        debug_usart_bind(USART1);

        // Send "Hello, World!" to PC
        debug_usart_print("Hello, World!\n");

        GPIOD->MODER = (1 << 24) | (1 << 26) | (1 << 28) | (1 << 30); // set pin 13 to be general purpose output
        uint8_t status = 0;

        for (;;) {
                GPIOD->ODR ^=  (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15); // Toggle the pin
                tmc5041_write_reg(tmc5041_GCONF, 0x00000008, &status);

                tmc5041_write_reg(tmc5041_GCONF, 0x000100c5, &status);

                int i;
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
                        tmc5041_write_reg(tmc5041_A1[i],         0x000013E8, &status);
                        tmc5041_write_reg(tmc5041_V1[i],         0x0001c350, &status);
                        tmc5041_write_reg(tmc5041_AMAX[i],       0x000011f4, &status);
                        tmc5041_write_reg(tmc5041_VMAX[i],       0x001304d0, &status);
                        tmc5041_write_reg(tmc5041_DMAX[i],       0x000012bc, &status);
                        tmc5041_write_reg(tmc5041_D1[i],         0x00001578, &status);
                        tmc5041_write_reg(tmc5041_VSTOP[i],      0x0000000A, &status);
                        tmc5041_write_reg(tmc5041_RAMPMODE[i],   0x00000000, &status);
                }

                int32_t location = 0;
                int32_t diff = 1000;

                uint32_t e = 0;
                while(1) {
                        tmc5041_write_reg(tmc5041_XTARGET[0], 0+e, &status);
                        tmc5041_write_reg(tmc5041_XTARGET[1], 0-e, &status);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        tmc5041_write_reg(tmc5041_XTARGET[0], 100000-e, &status);
                        tmc5041_write_reg(tmc5041_XTARGET[1], 0-e, &status);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        tmc5041_write_reg(tmc5041_XTARGET[0], 100000-e, &status);
                        tmc5041_write_reg(tmc5041_XTARGET[1], 100000+e, &status);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        tmc5041_write_reg(tmc5041_XTARGET[0], 0+e, &status);
                        tmc5041_write_reg(tmc5041_XTARGET[1], 100000+e, &status);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        tmc5041_write_reg(tmc5041_XTARGET[0], 0+e, &status);
                        tmc5041_write_reg(tmc5041_XTARGET[1], 0-e, &status);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        e += 10000;
                }

                while(1){
                        tmc5041_write_reg(tmc5041_XTARGET[0], location, &status);
                        tmc5041_write_reg(tmc5041_XTARGET[1], location, &status);
                        location += diff;
                        if(location < 0) {
                                diff = 1000;
                        }
                        if(location > 1000000) {
                                diff = -1000;
                        }
                        for(int i=0; i<5000000; i++) {__asm("nop");}
                        debug_usart_print("H\n");

                }
        }
}
/*
void pen_move_to(float x, float y)
{
        uint32_t A, uint32_t B;
        A = x + y;
        B = x - y;
        tmc5041_write_reg(MOTOR_XTARGET_0, A);
        tmc5041_write_reg(MOTOR_XTARGET_1, B);
}*/

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
                .reg = reg + 0x80,
                .data    = data
        };
        struct tmc5041_reply dummy;
        tmc5041_spi_transfer(&command, &dummy);
        *status = dummy.status;
}

uint32_t tmc5041_read_reg(uint8_t reg, uint32_t data, uint8_t *status)
{
        struct tmc5041_command command = {
                .reg = reg + 0x00,
                .data    = data
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
