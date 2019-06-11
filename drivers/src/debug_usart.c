#include "stm32f4xx_usart.h"
#include "debug_usart.h"

static USART_TypeDef *debug_usart = 0;

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

uint32_t debug_usart_print(char *s)
{
        if (debug_usart == 0) {
                return DEBUG_USART_ERR_NO_USART;
        }
        while (*s)
        {
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
