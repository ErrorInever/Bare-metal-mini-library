#ifndef BMML_RES_GPIO_H
#define BMML_RES_GPIO_H

#include "stm32f446xx.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define GPIO_PORT_RES_COUNT (sizeof(gpio_port_res)/sizeof(gpio_port_res[0]))
/* Resources layer for GPIO */


typedef struct {
    GPIO_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
    uint8_t exticr_code;            /* SYSCFG->EXTICR port selector: A=0,B=1,C=2,D=3,H=7 */
} gpio_port_res_t;


static const gpio_port_res_t gpio_port_res[] = {
    { GPIOA, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN, 0 },
    { GPIOB, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN, 1 },
    { GPIOC, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN, 2 },
    { GPIOD, &RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN, 3 },
    { GPIOH, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN, 7 },
};

// get index by gpio port
static inline int gpio_port_res_idx(const GPIO_TypeDef *port) {
    for(int i = 0; i < (int)GPIO_PORT_RES_COUNT; i++) 
        if(gpio_port_res[i].reg == port) return i;
    return -1;
}


static inline IRQn_Type gpio_exti_irqn(uint16_t pin) {
    if (pin <= 4)  return (IRQn_Type)(EXTI0_IRQn + pin);
    if (pin <= 9)  return EXTI9_5_IRQn;
    return EXTI15_10_IRQn;
}


#ifdef __cplusplus
}
#endif


#endif /* BMML_RES_GPIO_H */

