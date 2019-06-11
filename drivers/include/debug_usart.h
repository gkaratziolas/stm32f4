#ifndef DEBUG_USART_H_
#define DEBUG_USART_H_

#define DEBUG_USART_SUCCESS      0
#define DEBUG_USART_ERR_NO_USART 1

void debug_usart_bind(USART_TypeDef *USARTx);
void debug_usart_unbind(void);
void debug_usart_putchar(char c);
void debug_usart_print(char *s);
char debug_usart_getchar();

#endif // DEBUG_USART_H_
