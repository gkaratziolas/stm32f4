#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

// Included for fast floating point calculations
#include "math_utils.h"

#include "command_usart.h"
#include "debug_usart.h"
#include "pen_plotter.h"
#include "gpio.h"
#include "fifo.h"

/* Preprocessor defines and constants */
// USART command codes
#define COMMAND_RESET 0x01
#define COMMAND_LED   0x02
#define COMMAND_MOVE  0x03

#define MOTION_FIFO_LENGTH 10
struct int32_vec motion_fifo_array[MOTION_FIFO_LENGTH];
struct fifo motion_fifo;

/* Function prototypes */
// Peripheral Initialisation functions
void clock_init(void);
void io_init(void);
void usart_init(void);
void spi_init(void);
void nvic_init(void);

// Handler for incoming usart commands
void handle_command(struct command_packet *p);

int main(void)
{
        clock_init();
        io_init();
        spi_init();
        usart_init();
        nvic_init();
        command_usart_bind(USART1);
        debug_usart_bind(USART1);

        pen_motors_init();

        motion_fifo = fifo_init(motion_fifo_array,
                                sizeof(motion_fifo_array[0]),
                                MOTION_FIFO_LENGTH);

        struct command_packet p;

        GPIOD->ODR = 0x00;

        struct int32_vec start  = pen_get_position();
        struct int32_vec target = start;
        
        target.x = 10000000;
        target.y = 10000000;
        pen_set_target_position(target);
        while (1) {
                if (pen_motion_check_complete(start, target)) {
                        if (fifo_empty(&motion_fifo)) {
                                GPIOD->ODR &= ~(1<<13);
                                pen_stop();
                        } else {
                                start = pen_get_position();
                                fifo_pop(&motion_fifo, (void *) &target);
                                pen_set_target_position(target);
                                GPIOD->ODR |= (1<<13);
                        }
                }
                if (command_usart_receive(&p)) {
                        handle_command(&p);
                }
        }
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
                fifo_push(&motion_fifo, (void *) &target);
                break;
        }
        // Confirm packet was correctly interpreted by returning copy
        command_usart_transmit(p);
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
