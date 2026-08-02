#include "timer.h"
#include "bmml_res_timer.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stdbool.h>
#include <stdint.h>
#include "bmml_utilities.h"
#include "bmml_irq_shared.h"
#include <stdio.h>
#include <sys/types.h>


#define TIMER_BASIC_COUNT 2   // TIM6, TIM7
#define TIMER_PWM_COUNT 12    // TIM1..TIM5, TIM8..TIM14

static void timer6_isr_part(void);

// Basic timer pool
static timer_basic_t timer_basic_pool[TIMER_BASIC_COUNT];
static bool timer_basic_taken[TIMER_BASIC_COUNT];

static int timer_basic_slot(const TIM_TypeDef *reg) {
    if(reg == TIM6) return 0;
    if(reg == TIM7) return 1;
    return -1;
}

// PWM timer pool
static timer_pwm_t timer_pwm_pool[TIMER_PWM_COUNT];
static bool timer_pwm_taken[TIMER_PWM_COUNT];

static int timer_pwm_slot(const TIM_TypeDef *reg) {
    int idx = timer_res_idx(reg);
    if (idx < 0) return -1;
    if (timer_res[idx].category == TIMER_CAT_BASIC) return -1;
    if (idx <= 4) return idx;   // TIM1..TIM5 -> 0..4
    return idx - 2;             // TIM8..TIM14 (idx 7..13) -> 5..11
}

static uint32_t timer_get_clock_hz(const timer_res_t *res) {
    uint32_t pclk = res->on_apb2 ? get_apb2_clock_hz() : get_apb1_clock_hz();
    uint32_t ppre = res->on_apb2 ? ((RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x7)
                                  : ((RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7);
    return (ppre < 4) ? pclk : pclk * 2;
}

bmml_status_t timer_basic_acquire(TIM_TypeDef *reg, const uint32_t period_ms, timer_basic_callback_t cb, timer_basic_t **out) {
    if(out) *out = NULL;
    if (period_ms == 0 || period_ms > TIMER_BASIC_MAX_PERIOD_MS) return BMML_INVALID_ARG;

    int slot = timer_basic_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(timer_basic_taken[slot]) return BMML_BUSY;

    int idx = timer_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    // Register and init timer object
    timer_basic_taken[slot] = true;
    timer_basic_t *timer = &timer_basic_pool[slot];
    timer->res = &timer_res[idx];
    timer->callback = cb;

    // RCC
    *timer->res->rcc_reg |= timer->res->rcc_mask;
    // Configure PSC and ARR
    uint32_t freq = timer_get_clock_hz(timer->res);
    timer->res->reg->PSC = (freq / 10000U) - 1U;
    timer->res->reg->ARR = (period_ms * 10U) - 1U;
    // Forced update
    timer->res->reg->EGR |= TIM_EGR_UG;
    timer->res->reg->SR &= ~TIM_SR_UIF; // reset flag
    // Enable interrupt 
    timer->res->reg->DIER |= TIM_DIER_UIE;
    // Register specified IRQ for TIM6
    if(timer->res->reg == TIM6) {
        bmml_register_tim6_dac_handler(timer6_isr_part, NULL);
    }
    // Enable IRQ in NVIC
    NVIC_EnableIRQ(timer->res->irqn);


    if(out != NULL) {
        *out = timer;
    }

    return BMML_OK;
}

bmml_status_t timer_basic_release(timer_basic_t *timer) {
    if(timer == NULL || timer->res == NULL) return BMML_INVALID_ARG;

    TIM_TypeDef *reg  = timer->res->reg;
    IRQn_Type irqn = timer->res->irqn;

    int slot = timer_basic_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(!timer_basic_taken[slot]) return BMML_INVALID_ARG;  // double-release

    reg->CR1  &= ~TIM_CR1_CEN;
    reg->DIER &= ~TIM_DIER_UIE;

    if(reg == TIM6) bmml_unregister_tim6_dac_handler(true, false);
    NVIC_DisableIRQ(irqn);

    timer_basic_taken[slot] = false;
    timer_basic_pool[slot]  = (timer_basic_t){0};

    return BMML_OK;
}

bmml_status_t timer_basic_delay_poling(const timer_basic_t *timer, uint32_t ms) {
    TIM_TypeDef *reg = timer->res->reg;
    uint32_t wait_ticks = ms * 10U;
    if(wait_ticks > (uint32_t)(reg->ARR) + 1U) return BMML_INVALID_ARG; // delay is more than ARR

    bool was_disabled = !(reg->CR1 & TIM_CR1_CEN);
    if(was_disabled) reg->CR1 |= TIM_CR1_CEN;
    
    reg->DIER &= ~TIM_DIER_UIE; // temporarily disable interrupt while polling
    uint16_t start_tick = (uint16_t)reg->CNT;
    while((uint16_t)(reg->CNT - start_tick) < wait_ticks);
    reg->DIER |= TIM_DIER_UIE;

    if(was_disabled) reg->CR1 &= ~TIM_CR1_CEN;
    return BMML_OK;
}

// Basic timer ISR (TIM6 & TIM7)
static void timer6_isr_part(void) {
    if(!timer_basic_taken[0]) return;
    if(TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF;
        if(timer_basic_pool[0].callback != NULL) {
            timer_basic_pool[0].callback();
        }
    }
}
// TIM7 handler
void TIM7_IRQHandler(void) { 
    if(!timer_basic_taken[1]) return;
    if(TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;
        if(timer_basic_pool[1].callback != NULL) {
            timer_basic_pool[1].callback();
        }
    }
}


bmml_status_t timer_pwm_acquire(TIM_TypeDef *reg, uint32_t freq_hz, timer_pwm_t **out) {
    if(out) *out = NULL;
    if(freq_hz == 0) return BMML_INVALID_ARG;

    int slot = timer_pwm_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(timer_pwm_taken[slot]) return BMML_BUSY;

    int idx = timer_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    const timer_res_t *res = &timer_res[idx];
    uint32_t timer_clk = timer_get_clock_hz(res);   // not need RCC-enabled
    uint32_t arr_max = res->is_32bit ? 0xFFFFFFFFU : 0xFFFFU;

    if(freq_hz > timer_clk) return BMML_INVALID_ARG;  // freq not reacheable

    uint32_t psc = 0, arr = 0;
    bool found = false;
    do {
        uint64_t denom = (uint64_t)(psc + 1U) * freq_hz;   // uint64_t — without overflow
        uint64_t candidate = timer_clk / denom;
        if (candidate > 0 && candidate <= arr_max) { arr = (uint32_t)candidate; found = true; break; }
        psc++;
    } while (psc <= 0xFFFFU);

    if (!found) return BMML_INVALID_ARG;

    timer_pwm_taken[slot] = true;
    timer_pwm_t *timer = &timer_pwm_pool[slot];
    timer->res = res;

    *res->rcc_reg |= res->rcc_mask;
    res->reg->PSC = psc;
    res->reg->ARR = arr - 1U;
    res->reg->EGR |= TIM_EGR_UG;
    res->reg->SR &= ~TIM_SR_UIF;

    if(out) *out = timer;
    return BMML_OK;
}

bmml_status_t timer_pwm_release(timer_pwm_t *timer) {
    if(timer == NULL || timer->res == NULL) return BMML_INVALID_ARG;

    TIM_TypeDef *reg  = timer->res->reg;

    int slot = timer_pwm_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(!timer_pwm_taken[slot]) return BMML_INVALID_ARG;  

    reg->CR1 &= ~TIM_CR1_CEN;

    for(int ch = 0; ch < 4; ch++) {
        if(timer->channel_enabled_mask & (1U << ch)) {
            reg->CCER &= ~channel_enable[ch];
        }
    }

    if(reg == TIM1 || reg == TIM8) {
        reg->BDTR &= ~TIM_BDTR_MOE;
    }

    timer_pwm_taken[slot] = false;
    timer_pwm_pool[slot] = (timer_pwm_t){0};

    return BMML_OK;
}

bmml_status_t timer_pwm_channel_config(timer_pwm_t *timer, uint8_t channel, uint32_t duty) {
    if(timer == NULL || timer->res == NULL) return BMML_INVALID_ARG;
    if(channel < 1 || channel > timer->res->num_channels) return BMML_INVALID_ARG;
    if(timer->channel_enabled_mask & (1U << (channel - 1))) return BMML_BUSY;
    if(duty > 100U) duty = 100U;

    TIM_TypeDef *reg  = timer->res->reg;

    uint32_t ccr_val = ((reg->ARR + 1U) * duty) / 100U;
    
    switch (channel) {
        case 1: reg->CCR1 = ccr_val; reg->CCMR1 = (reg->CCMR1 & ~TIM_CCMR1_OC1M) | (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; break;
        case 2: reg->CCR2 = ccr_val; reg->CCMR1 = (reg->CCMR1 & ~TIM_CCMR1_OC2M) | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; break;
        case 3: reg->CCR3 = ccr_val; reg->CCMR2 = (reg->CCMR2 & ~TIM_CCMR2_OC3M) | (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; break;
        case 4: reg->CCR4 = ccr_val; reg->CCMR2 = (reg->CCMR2 & ~TIM_CCMR2_OC4M) | (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; break;
    }

    timer->channel_enabled_mask |= (1U << (channel - 1));

    // If this TIM1 or TIM8, need enable main output
    if(reg == TIM1 || reg == TIM8) reg->BDTR |= TIM_BDTR_MOE;

    return BMML_OK;
}

bmml_status_t timer_pwm_channel_enable(const timer_pwm_t *timer, uint8_t channel) {
    if(timer == NULL || timer->res == NULL || channel < 1 || channel > 4) return BMML_INVALID_ARG;
    if(!(timer->channel_enabled_mask & (1U << (channel - 1)))) return BMML_ERROR;

    timer->res->reg->CCER |= channel_enable[channel - 1];

    return BMML_OK;
}

bmml_status_t timer_pwm_channel_disable(const timer_pwm_t *timer, uint8_t channel) {
    if(timer == NULL || timer->res == NULL || channel < 1 || channel > 4) return BMML_INVALID_ARG;
    if(!(timer->channel_enabled_mask & (1U << (channel - 1)))) return BMML_ERROR;

    timer->res->reg->CCER &= ~channel_enable[channel - 1];

    return BMML_OK;
}

bmml_status_t timer_pwm_set_duty(const timer_pwm_t *timer, uint8_t channel, uint32_t duty) {
    if(timer == NULL || timer->res == NULL || channel < 1 || channel > 4) return BMML_INVALID_ARG;
    if(!(timer->channel_enabled_mask & (1U << (channel - 1)))) return BMML_ERROR;
    if(duty > 100U) duty = 100U;

    uint32_t ccr_val = ((timer->res->reg->ARR + 1U) * duty) / 100U;
    switch(channel) {
        case 1: timer->res->reg->CCR1 = ccr_val; break;
        case 2: timer->res->reg->CCR2 = ccr_val; break;
        case 3: timer->res->reg->CCR3 = ccr_val; break;
        case 4: timer->res->reg->CCR4 = ccr_val; break;
    }

    return BMML_OK;
}