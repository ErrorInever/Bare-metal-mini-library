#include "gpio.h"
#include "bmml_status.h"
#include <stddef.h>
#include "bmml_res_gpio.h"




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

    exti_slot_t *slot = &exti_slots[gpio->pin];

    // Line is still busy another PORT
    if(slot->owner_port != NULL && slot->owner_port != gpio->port) return BMML_BUSY;

    slot->owner_port = gpio->port;
    slot->cb = callback;

    return BMML_OK;
}