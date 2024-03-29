#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "command_usart.h"

#define COMMAND_RESET 0x01
#define COMMAND_LED   0x02

void clock_init(void);
void io_init(void);
void usart_init(void);
void nvic_init(void);
void handle_command(struct command_packet *p);

int main(void)
{
        clock_init();
        io_init();
        usart_init();
        nvic_init();
        command_usart_bind(USART1);

        struct command_packet p;
        
        while(1) {
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
        }
        // Confirm packet was correctly interpreted by returning copy
        command_usart_transmit(p);
}

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
