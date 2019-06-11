#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#include "debug_usart.h"


#define MOTOR_GSTAT_REG 0x01

#define MOTOR_RAMPMODE_REG_0 0x20
#define MOTOR_RAMPMODE_REG_1 0x40

#define MOTOR_A1_REG_0 0x24
#define MOTOR_A1_REG_1 0x44
#define MOTOR_V1_REG_0 0x25
#define MOTOR_V1_REG_1 0x45
#define MOTOR_AMAX_REG_0 0x26
#define MOTOR_AMAX_REG_1 0x46
#define MOTOR_VMAX_REG_0 0x27
#define MOTOR_VMAX_REG_1 0x47
#define MOTOR_DMAX_REG_0 0x28
#define MOTOR_DMAX_REG_1 0x48
#define MOTOR_D1_REG_0 0x2a
#define MOTOR_D1_REG_1 0x4a
#define MOTOR_VSTOP_REG_0 0x2b
#define MOTOR_VSTOP_REG_1 0x4b
#define MOTOR_TZEROWAIT_REG_0 0x2c
#define MOTOR_TZEROWAIT_REG_1 0x4c
#define MOTOR_XTARGET_0 0x2d
#define MOTOR_XTARGET_1 0x4d

#define MOTOR_IHOLD_IRUN_REG_0 0x30
#define MOTOR_IHOLD_IRUN_REG_1 0x50
#define MOTOR_VCOOLTHRS_REG_0  0x31
#define MOTOR_VCOOLTHRS_REG_1  0x51

#define MOTOR_VHIGH_REG_0  0x32
#define MOTOR_VHIGH_REG_1  0x52

#define MOTOR_GCONF_REG 0x00

#define MOTOR_CHOPCONF_REG_0 0x6c
#define MOTOR_CHOPCONF_REG_1 0x7c

#define MOTOR_PWMCONF_REG_0 0x10
#define MOTOR_PWMCONF_REG_1 0x18


void clock_init(void);
void io_init(void);
void usart_init(void);
void spi_init(void);
void spi_send_byte(uint8_t byte);
void gpio_set(uint8_t pin, GPIO_TypeDef* port);
void gpio_clear(uint8_t pin, GPIO_TypeDef* port);
void spi_send_motor_command(uint8_t addr, uint32_t data);
void motor_write_reg(uint8_t reg, uint32_t command);

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

        for (;;) {
                GPIOD->ODR ^=  (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15); // Toggle the pin
                motor_write_reg(MOTOR_GCONF_REG, 0x00000008);

                motor_write_reg(MOTOR_CHOPCONF_REG_0, 0x000100c5);
                motor_write_reg(MOTOR_IHOLD_IRUN_REG_0, 0x00011f05);
                motor_write_reg(MOTOR_TZEROWAIT_REG_0, 0x00002710);
                motor_write_reg(MOTOR_PWMCONF_REG_0, 0x000401c8);
                motor_write_reg(MOTOR_VHIGH_REG_0, 0x00061a80);
                motor_write_reg(MOTOR_VCOOLTHRS_REG_0, 0x00007530);

                motor_write_reg(MOTOR_CHOPCONF_REG_1, 0x000100c5);
                motor_write_reg(MOTOR_IHOLD_IRUN_REG_1, 0x00011f05);
                motor_write_reg(MOTOR_TZEROWAIT_REG_1, 0x00002710);
                motor_write_reg(MOTOR_PWMCONF_REG_1, 0x000401c8);
                motor_write_reg(MOTOR_VHIGH_REG_1, 0x00061a80);
                motor_write_reg(MOTOR_VCOOLTHRS_REG_1, 0x00007530);

                motor_write_reg(MOTOR_A1_REG_0, 0x000013E8);
                motor_write_reg(MOTOR_V1_REG_0, 0x0001c350);
                motor_write_reg(MOTOR_AMAX_REG_0, 0x000011f4);
                motor_write_reg(MOTOR_VMAX_REG_0, 0x001304d0);
                motor_write_reg(MOTOR_DMAX_REG_0, 0x000012bc);
                motor_write_reg(MOTOR_D1_REG_0, 0x00001578);
                motor_write_reg(MOTOR_VSTOP_REG_0, 0x0000000A);
                motor_write_reg(MOTOR_RAMPMODE_REG_0, 0x00000000);

                motor_write_reg(MOTOR_A1_REG_1, 0x000013E8);
                motor_write_reg(MOTOR_V1_REG_1, 0x0001c350);
                motor_write_reg(MOTOR_AMAX_REG_1, 0x000011f4);
                motor_write_reg(MOTOR_VMAX_REG_1, 0x001304d0);
                motor_write_reg(MOTOR_DMAX_REG_1, 0x000012bc);
                motor_write_reg(MOTOR_D1_REG_1, 0x00001578);
                motor_write_reg(MOTOR_VSTOP_REG_1, 0x0000000A);
                motor_write_reg(MOTOR_RAMPMODE_REG_1, 0x00000000);


                int32_t location = 0;
                int32_t diff = 1000;

                uint32_t e = 0;
                while(1) {
                        motor_write_reg(MOTOR_XTARGET_0, 0+e);
                        motor_write_reg(MOTOR_XTARGET_1, 0-e);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        motor_write_reg(MOTOR_XTARGET_0, 100000-e);
                        motor_write_reg(MOTOR_XTARGET_1, 0-e);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        motor_write_reg(MOTOR_XTARGET_0, 100000-e);
                        motor_write_reg(MOTOR_XTARGET_1, 100000+e);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        motor_write_reg(MOTOR_XTARGET_0, 0+e);
                        motor_write_reg(MOTOR_XTARGET_1, 100000+e);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        motor_write_reg(MOTOR_XTARGET_0, 0+e);
                        motor_write_reg(MOTOR_XTARGET_1, 0-e);
                        for(int i=0; i<50000000; i++) {__asm("nop");}
                        e += 10000;
                }

                while(1){
                        motor_write_reg(MOTOR_XTARGET_0, location);
                        motor_write_reg(MOTOR_XTARGET_1, location);
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
        motor_write_reg(MOTOR_XTARGET_0, A);
        motor_write_reg(MOTOR_XTARGET_1, B);
}*/

void clock_init(void)
{
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

        RCC->AHB1ENR |= (
                RCC_AHB1ENR_GPIOAEN |  // enable the clock to GPIOA
                RCC_AHB1ENR_GPIODEN    // enable the clock to GPIOD
        );
        __asm("dsb");  // stall instruction pipeline, until instruction completes, as
                       // per Errata 2.1.13, "Delay after an RCC peripheral clock enabling"

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

void gpio_set(uint8_t pin, GPIO_TypeDef* port) {
        port->ODR |= (1<<pin);
}

void gpio_clear(uint8_t pin, GPIO_TypeDef* port) {
        port->ODR &= ~(1<<pin);
}

void gpio_toggle(uint8_t pin, GPIO_TypeDef* port) {
        port->ODR ^= (1<<pin);
}

void spi_init(void) {
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

void spi_send_byte_blocking(uint8_t byte) {
        gpio_clear(3, GPIOA);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        SPI1->DR = byte;
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        gpio_set(3, GPIOA);
}

// TODO: Two while loops are needed to check the BSY flag! 
void spi_send_motor_command(uint8_t addr, uint32_t data) {
        gpio_clear(4, GPIOA);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        SPI1->DR = addr;
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        uint8_t status = SPI1->DR;
        SPI1->DR = ((data & 0xff000000) >> 24);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        SPI1->DR = ((data & 0x00ff0000) >> 16);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        SPI1->DR = ((data & 0x0000ff00) >> 8);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        SPI1->DR = ((data & 0x000000ff) >> 0);
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        while(SPI1->SR & SPI_SR_BSY) {__asm("nop");}
        gpio_set(4, GPIOA);
}

void motor_write_reg(uint8_t reg, uint32_t command) {
      spi_send_motor_command(reg+0x80, command);  
}

void motor_read_reg(uint8_t reg, uint32_t command) {
      spi_send_motor_command(reg, command);  
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
