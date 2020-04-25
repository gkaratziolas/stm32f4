#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f4xx.h"

// GPIO commands - to be moved to a driver
void gpio_set   (uint8_t pin, GPIO_TypeDef* port);
void gpio_clear (uint8_t pin, GPIO_TypeDef* port);
void gpio_toggle(uint8_t pin, GPIO_TypeDef* port);

#endif // GPIO_H_
