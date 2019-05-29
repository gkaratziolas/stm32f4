#include "stm32f4xx_usart.h"
#include "debug_usart.h"

static USART_TypeDef *debug_uart = 0;

void debug_usart_bind(USART_TypeDef *UARTx)
{
        debug_uart = UARTx;      
}

void debug_usart_unbind(void)
{
        debug_uart = 0;      
}

void debug_usart_putchar(char c)
{
        // Wait until transmit data register is empty
        while (!USART_GetFlagStatus(debug_uart, USART_FLAG_TXE));
        // Send a char using debug_uart
        USART_SendData(debug_uart, c);
}

void debug_usart_print(char *s)
{
        while (*s)
        {
                debug_usart_putchar(*s++);
        }
        // Print carriage return
        debug_usart_putchar('\r');
}

char debug_usart_getchar()
{
        char c;
        // Wait until data is received
        while (!USART_GetFlagStatus(debug_uart, USART_FLAG_RXNE));
        // Read received char
        c = USART_ReceiveData(debug_uart);
        debug_usart_putchar(c);
        return c;
}
