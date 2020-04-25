#include "stm32f4xx_usart.h"
#include "system_utils.h"
#include "debug_usart.h"

#include <stdio.h>

static USART_TypeDef *debug_usart = 0;

void debug_usart_print_timestamp(void);

void debug_usart_bind(USART_TypeDef *USARTx)
{
        debug_usart = USARTx;
}

void debug_usart_unbind(void)
{
        debug_usart = 0;      
}

uint32_t debug_usart_putchar(char c)
{
        if (debug_usart == 0) {
                return DEBUG_USART_ERR_NO_USART;
        }
        // Wait until transmit data register is empty
        while (!USART_GetFlagStatus(debug_usart, USART_FLAG_TXE));
        // Send a char using debug_usart
        USART_SendData(debug_usart, c);
        return DEBUG_USART_SUCCESS;
}

void debug_usart_print_timestamp(void)
{
        char time_str[9];
        uint32_t i;
        uint32_t uptime_ms = system_uptime_ms_get();
        uint32_t show_char = 0;

        sprintf(time_str, "%08lu", uptime_ms);

        debug_usart_putchar('[');
        for (i=0; i<8; i++) {
                if ((time_str[i] != '0') || (i == 4)) {
                        show_char = 1;
                }
                if (i == 5) {
                        debug_usart_putchar('.');
                }
                if (show_char) {
                        debug_usart_putchar(time_str[i]);
                } else {
                        debug_usart_putchar(' ');
                }
        }
        debug_usart_putchar(']');
        debug_usart_putchar(' ');
}

uint32_t debug_usart_print(char *s)
{
        if (debug_usart == 0) {
                return DEBUG_USART_ERR_NO_USART;
        }
        if (DEBUG_USART_PRINT_TIMESTAMP) {
                debug_usart_print_timestamp();
        }
        while (*s) {
                debug_usart_putchar(*s++);
        }
        // Print carriage return
        debug_usart_putchar('\r');
        return DEBUG_USART_SUCCESS;
}

char debug_usart_getchar()
{
        char c;
        // Wait until data is received
        while (!USART_GetFlagStatus(debug_usart, USART_FLAG_RXNE));
        // Read received char
        c = USART_ReceiveData(debug_usart);
        debug_usart_putchar(c);
        return c;
}
