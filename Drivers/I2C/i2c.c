#include "i2c.h"
#include "bmml_res_i2c.h"
#include "bmml_status.h"
#include "bmml_utilities.h"
#include "stm32f446xx.h"
#include <stdbool.h>
#include <stdint.h>

#define I2C_COUNT 3

static i2c_t i2c_pool[I2C_COUNT];
static bool i2c_taken[I2C_COUNT];

static int i2c_slot(const I2C_TypeDef *reg) {
    if(reg == I2C1) return 0;
    if(reg == I2C2) return 1;
    if(reg == I2C3) return 2;
    return -1;
}

bmml_status_t i2c_it_acquire(I2C_TypeDef *reg, i2c_mode_t mode, i2c_callback_t cb, i2c_t **out) {
    if(out) *out = NULL;
    if(reg == NULL) return BMML_INVALID_ARG;
    if(mode != I2C_SM_100KHZ && mode != I2C_FM_400KHZ) return BMML_INVALID_ARG;

    int slot = i2c_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(i2c_taken[slot]) return BMML_BUSY;

    int idx = i2c_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    // Init I2C object
    i2c_taken[slot] = true;
    i2c_t *i2c = &i2c_pool[slot];
    i2c->res = &i2c_res[idx];
    i2c->callback = cb;

    // RCC
    *i2c->res->rcc_reg |= i2c->res->rcc_mask;

    i2c_reset_off(reg);

    // Configure freq
    uint32_t freq = get_apb1_clock_hz();
    i2c->res->reg->CR2 &= ~I2C_CR2_FREQ;
    i2c->res->reg->CR2 |= (freq & I2C_CR2_FREQ);

    // reset CCR & TRISE
    i2c->res->reg->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR);
    i2c->res->reg->TRISE &= ~I2C_TRISE_TRISE;

    if(mode == I2C_SM_100KHZ) {
        // Standard Mode (100 kHz)
        // CCR = F_pclk1 / (2 * 100 000) -> multiple 1 000 000 and then devide by 1000
        uint32_t ccr_val = (freq * 1000U) / 200U;
        if (ccr_val < 4) ccr_val = 4; // The minimum allowed value is 0x04, except when Duty = 1 (c)
        i2c->res->reg->CCR |= (ccr_val & I2C_CCR_CCR);
        // Calculate TRISE
        // TRISE = (1000ns / pclk1_mhz) + 1
        i2c->res->reg->TRISE |= ((freq + 1U) & I2C_TRISE_TRISE);
    } else {
        // Fast Mode (400 kHz)
        // Enable Fast mode bit and setup DUTY = 0. t_low/t_high = 2
        i2c->res->reg->CCR |= I2C_CCR_FS;
        // For DUTY = 0: CCR = F_pclk1 / (3 * 400 000)
        uint32_t ccr_val = (freq * 1000U) / 1200U;
        if (ccr_val < 1) ccr_val = 1;
        i2c->res->reg->CCR |= (ccr_val & I2C_CCR_CCR);
        // TRISE: For Fast Mode max time rise 300ns
        // (300ns * F_pclk1) + 1 = (0.3 * pclk1_mhz) + 1
        uint32_t trise_val = ((freq * 300U) / 1000U) + 1U;
        i2c->res->reg->TRISE |= (trise_val & I2C_TRISE_TRISE);
    }

    // Enable interrupts
    NVIC_EnableIRQ(i2c->res->irqn_ev);
    NVIC_SetPriority(i2c->res->irqn_ev, 5);
    NVIC_EnableIRQ(i2c->res->irqn_er);
    NVIC_SetPriority(i2c->res->irqn_er, 5);
    i2c_enable(reg);

    return BMML_OK;
}