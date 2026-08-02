#ifndef BMML_RES_TIMER_H
#define BMML_RES_TIMER_H


#ifdef __cplusplus
extern "C" {
#endif

#define TIMER_RES_COUNT (sizeof(timer_res)/sizeof(timer_res[0]))

#include <stdint.h>
#include "stm32f446xx.h"
#include "stdbool.h"

typedef enum {
    TIMER_CAT_BASIC,
    TIMER_CAT_GP,
    TIMER_CAT_ADVANCED
} timer_category_t;

static const uint32_t channel_enable[4] = {TIM_CCER_CC1E, TIM_CCER_CC2E, TIM_CCER_CC3E, TIM_CCER_CC4E};

typedef struct {
    TIM_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
    IRQn_Type irqn;
    timer_category_t category;
    uint8_t num_channels;
    bool is_32bit;
    bool on_apb2;
} timer_res_t;


static const timer_res_t timer_res[] = {
    {  TIM1,  &RCC->APB2ENR, RCC_APB2ENR_TIM1EN,  TIM1_CC_IRQn,             TIMER_CAT_ADVANCED, 4, false, true  },
    {  TIM2,  &RCC->APB1ENR, RCC_APB1ENR_TIM2EN,  TIM2_IRQn,                TIMER_CAT_GP,       4, true,  false },
    {  TIM3,  &RCC->APB1ENR, RCC_APB1ENR_TIM3EN,  TIM3_IRQn,                TIMER_CAT_GP,       4, false, false },
    {  TIM4,  &RCC->APB1ENR, RCC_APB1ENR_TIM4EN,  TIM4_IRQn,                TIMER_CAT_GP,       4, false, false },
    {  TIM5,  &RCC->APB1ENR, RCC_APB1ENR_TIM5EN,  TIM5_IRQn,                TIMER_CAT_GP,       4, true,  false },
    {  TIM6,  &RCC->APB1ENR, RCC_APB1ENR_TIM6EN,  TIM6_DAC_IRQn,            TIMER_CAT_BASIC,    0, false, false },
    {  TIM7,  &RCC->APB1ENR, RCC_APB1ENR_TIM7EN,  TIM7_IRQn,                TIMER_CAT_BASIC,    0, false, false },
    {  TIM8,  &RCC->APB2ENR, RCC_APB2ENR_TIM8EN,  TIM8_CC_IRQn,             TIMER_CAT_ADVANCED, 4, false, true  },
    {  TIM9,  &RCC->APB2ENR, RCC_APB2ENR_TIM9EN,  TIM1_BRK_TIM9_IRQn,       TIMER_CAT_GP,       2, false, true  },
    {  TIM10, &RCC->APB2ENR, RCC_APB2ENR_TIM10EN, TIM1_UP_TIM10_IRQn,       TIMER_CAT_GP,       1, false, true  },
    { TIM11, &RCC->APB2ENR, RCC_APB2ENR_TIM11EN, TIM1_TRG_COM_TIM11_IRQn,  TIMER_CAT_GP,        1, false, true  },
    { TIM12, &RCC->APB1ENR, RCC_APB1ENR_TIM12EN, TIM8_BRK_TIM12_IRQn,      TIMER_CAT_GP,        2, false, false },
    { TIM13, &RCC->APB1ENR, RCC_APB1ENR_TIM13EN, TIM8_UP_TIM13_IRQn,       TIMER_CAT_GP,        1, false, false },
    { TIM14, &RCC->APB1ENR, RCC_APB1ENR_TIM14EN, TIM8_TRG_COM_TIM14_IRQn,  TIMER_CAT_GP,        1, false, false },
};

// Get index of timer res
static inline int timer_res_idx(const TIM_TypeDef *reg) {
    for(int i = 0; i < (int)TIMER_RES_COUNT; i++) {
        if(timer_res[i].reg == reg) return i;
    }
    return -1;
}

// TODO IRQn type


#ifdef __cplusplus
}
#endif


#endif /* BMML_RES_TIMER_H */
