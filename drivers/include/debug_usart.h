#ifndef DEBUG_USART_H_
#define DEBUG_USART_H_

void debug_usart_bind(USART_TypeDef *USARTx);
void debug_usart_unbind(void);
void debug_usart_putchar(char c);
void debug_usart_print(char *s);
char debug_usart_getchar();

#endif // DEBUG_USART_H_
