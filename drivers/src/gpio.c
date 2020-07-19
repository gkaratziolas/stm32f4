#include "gpio.h"
#include "stm32f4xx_gpio.h"

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
