#include "gpio.h"
#include "bmml_status.h"
#include <stddef.h>
#include <stdint.h>
#include "bmml_res_gpio.h"
#include "stm32f446xx.h"


// EXTI lines. Each line has callback
exti_slot_t exti_slots[16] = {0};

static inline bmml_status_t exti_claim_line(const gpio_t *gpio) {
    exti_slot_t *slot = &exti_slots[gpio->pin];
    // The line is busy with another port
    if(slot->owner_port != NULL && slot->owner_port != gpio->port)
        return BMML_BUSY;

    slot->owner_port = gpio->port;
    return BMML_OK;
}

bmml_status_t gpio_init(const gpio_t *gpio, gpio_mode_t mode, gpio_pull_t pull, gpio_speed_t speed, gpio_otype_t otype) {
    if(gpio == NULL || gpio->pin > 15) return BMML_INVALID_ARG;

    int idx = gpio_port_res_idx(gpio->port);
    if (idx < 0) return BMML_INVALID_ARG;   // port doesn't exist in current MCU

    // Enable RCC
    *gpio_port_res[idx].rcc_reg |= gpio_port_res[idx].rcc_mask;

    uint32_t pin_pos = gpio->pin * 2U;
    // Set pin mode
    gpio->port->MODER &= ~(3U << pin_pos);
    gpio->port->MODER |= ((uint32_t)mode << pin_pos);
    // Pull
    gpio->port->PUPDR &= ~(3U << pin_pos);
    gpio->port->PUPDR |= ((uint32_t)pull << pin_pos);
    // Output type (requiered 1 bit)
    gpio->port->OTYPER &= ~(1U << gpio->pin);
    gpio->port->OTYPER |= ((uint32_t)otype << gpio->pin);
    // Output speed
    gpio->port->OSPEEDR &= ~(3U << pin_pos);
    gpio->port->OSPEEDR |= ((uint32_t)speed << pin_pos);

    return BMML_OK;
}

bmml_status_t gpio_set_exti_callback(const gpio_t *gpio, gpio_callback_t callback) {
    if(gpio == NULL || gpio->pin > 15) return BMML_INVALID_ARG;
    bmml_status_t st = exti_claim_line(gpio);

    if(st != BMML_OK) return st;

    exti_slots[gpio->pin].cb = callback;
    return BMML_OK;
}

bmml_status_t gpio_set_alternate_func(const gpio_t *gpio, uint8_t af_num) {
    if(gpio == NULL || gpio->pin > 15 || af_num > 15) return BMML_INVALID_ARG;

    uint8_t reg_idx = gpio->pin >> 3U;              // (pin // 8), if 0..7 = 0, if 8..15 = 1
    uint8_t bit_pos = (gpio->pin & 0x07U) * 4U;     // pin % 0x07 * 4byte. 10pin & 0x07 = 2 -> 2 * 4 = 8 -> 8,9,10,11 
    gpio->port->AFR[reg_idx] &= ~(0x0FU << bit_pos);
    gpio->port->AFR[reg_idx] |= ((uint32_t)af_num << bit_pos);

    return BMML_OK;
}

bmml_status_t gpio_enable_exti_isr(const gpio_t *gpio, edge_type_t edge) {
    if(gpio == NULL || gpio->pin > 15) return BMML_INVALID_ARG;

    int idx = gpio_port_res_idx(gpio->port);
    if (idx < 0) return BMML_INVALID_ARG;   // port doesn't exist in current MCU

    bmml_status_t st = exti_claim_line(gpio);
    if(st != BMML_OK) return st;

    // Enable RCC SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    // Сonnect the EXTI N line to the port M with SYSCFG
    uint8_t exticr_idx = gpio->pin >> 2U;   // each pin 4 bite
    uint8_t pos = (gpio->pin & 0x03U) * 4U;

    SYSCFG->EXTICR[exticr_idx] &= ~(0x0FU << pos);
    SYSCFG->EXTICR[exticr_idx] |= (gpio_port_res[idx].exticr_code << pos);

    // IMR Enable interrupt for EXTI N line
    EXTI->IMR |= (1U << gpio->pin);

    // Select and enable edge trigger
    if(edge == RISING_EDGE || edge == RISING_FALLING) EXTI->RTSR  |= (1U << gpio->pin);
    if(edge == FALLING_EDGE || edge == RISING_FALLING) EXTI->FTSR |= (1U << gpio->pin);

    // Enable interrupt in NVIC
    NVIC_EnableIRQ(gpio_exti_irqn(gpio->pin));

    return BMML_OK;
}

// EXTI ISR
static void gpio_isr_handle(uint16_t pin) {
    if(EXTI->PR & (1U << pin)) {
        EXTI->PR = (1U << pin);     // reset flag
        if(exti_slots[pin].cb != NULL) {
            gpio_t g = { .port = (GPIO_TypeDef *)exti_slots[pin].owner_port, .pin = pin};
            exti_slots[pin].cb(&g);
        } 
    }
}

// IRQHandlers
void EXTI0_IRQHandler(void) { gpio_isr_handle(0); }
void EXTI1_IRQHandler(void) { gpio_isr_handle(1); }
void EXTI2_IRQHandler(void) { gpio_isr_handle(2); }
void EXTI3_IRQHandler(void) { gpio_isr_handle(3); }
void EXTI4_IRQHandler(void) { gpio_isr_handle(4); }
void EXTI9_5_IRQHandler(void) { for(int i = 5; i <= 9; i++) gpio_isr_handle(i); }
void EXTI15_10_IRQHandler(void) { for(int i = 10; i <= 15; i++) gpio_isr_handle(i); }