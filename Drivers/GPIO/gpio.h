#ifndef GPIO_H
#define GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdbool.h>

/** * @struct gpio_t
 * @brief GPIO Pin Descriptor.
 *
 * Encapsulates a single hardware pin reference on a specific port.
 */
typedef struct {
    GPIO_TypeDef *port;   /**< Base register pointer of the target GPIO peripheral (GPIOA, GPIOB, etc.). */
    uint16_t pin;         /**< Pin index number (Valid range: 0 to 15). */
} gpio_t;

// User callback definition
typedef bmml_status_t (*gpio_callback_t)(const gpio_t *gpio);


typedef struct {
    gpio_callback_t cb;
    const GPIO_TypeDef *owner_port;
} exti_slot_t;

// EXTI lines. each line has callback
static exti_slot_t exti_slots[16] = {0};

/** * @enum gpio_mode_t
 * @brief Standard hardware pin operating modes.
 */
typedef enum {
    GPIO_MODE_INPUT  = 0x00,    /**< Digital Input Mode. */
    GPIO_MODE_OUTPUT = 0x01,    /**< Digital Push-Pull / Open-Drain Output Mode. */
    GPIO_MODE_AF     = 0x02,    /**< Alternate Function Multiplexer Mode (e.g., UART, SPI, I2C). */
    GPIO_MODE_ANALOG = 0x03     /**< Analog Input/Output Mode for ADC or DAC isolation. */
} gpio_mode_t;


/** * @enum gpio_pull_t
 * @brief Internal pull-up or pull-down resistor electrical configurations.
 */
typedef enum {
    GPIO_PULL_NONE    = 0x00,   /**< No internal resistor connection (Floating). */
    GPIO_PULL_UP      = 0x01,   /**< Internal weak pull-up resistor engaged. */
    GPIO_PULL_DOWN    = 0x02    /**< Internal weak pull-down resistor engaged. */
} gpio_pull_t;

/** * @enum gpio_speed_t
 * @brief Output driver slew rate / frequency response controls.
 */
typedef enum {
    GPIO_SPEED_LOW          = 0x00,     /**< Low speed (Max 2 MHz - minimal electrical noise). */
    GPIO_SPEED_MEDIUM       = 0x01,     /**< Medium speed (Max 12.5 MHz to 50 MHz). */
    GPIO_SPEED_HIGH         = 0x02,     /**< Fast speed (Max 25 MHz to 100 MHz). */
    GPIO_SPEED_VERY_HIGH    = 0x03      /**< High speed (Max 50 MHz to 200 MHz - critical for high-speed buses). */
} gpio_speed_t;

/** * @enum gpio_otype_t
 * @brief Output driver electrical configuration types.
 */
typedef enum {
    GPIO_OTYPE_PP   = 0x00,     /**< Push-Pull output stage (Actively drives both High and Low levels). */
    GPIO_OTYPE_OD   = 0x01      /**< Open-Drain output stage (Requires external pull-up for High level). */
} gpio_otype_t;

/** * @enum edge_type_t
 * @brief Edge trigger detection conditions for EXTI external hardware interrupts.
 */
typedef enum {
    RISING_EDGE,        /**< Interrupt triggers exclusively on a Low-to-High signal transition. */
    FALLING_EDGE,       /**< Interrupt triggers exclusively on a High-to-Low signal transition. */
    RISING_FALLING      /**< Interrupt triggers on both signal state transitions. */
} edge_type_t;


bmml_status_t gpio_init(const gpio_t *gpio, gpio_mode_t mode, gpio_pull_t pull, gpio_speed_t speed, gpio_otype_t otype);
bmml_status_t gpio_set_exti_callback(const gpio_t *gpio, gpio_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /*GPIO_H*/
