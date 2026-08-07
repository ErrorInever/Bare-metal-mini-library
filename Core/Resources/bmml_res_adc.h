#ifndef BMML_RES_ADC_H
#define BMML_RES_ADC_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"

#define ADC_RES_COUNT 3

typedef struct {
    ADC_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
} adc_res_t;

static const adc_res_t adc_res[] = {
    {ADC1, &RCC->APB2ENR, RCC_APB2ENR_ADC1EN},
    {ADC2, &RCC->APB2ENR, RCC_APB2ENR_ADC2EN},
    {ADC3, &RCC->APB2ENR, RCC_APB2ENR_ADC3EN},
};

// Get index of ADC res
static inline int dac_res_idx(const ADC_TypeDef *reg) {
    for(int i = 0; i < (int)ADC_RES_COUNT; i++) {
        if(adc_res[i].reg == reg) return i;
    }
    return -1;
}


#ifdef __cplusplus
}
#endif

#endif /*BMML_RES_ADC_H*/