#ifndef BMML_RES_DAC_H
#define BMML_RES_DAC_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"


#define DAC_RES_COUNT 1

typedef struct {
    DAC_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
} dac_res_t;

static const dac_res_t dac_res[] = {
    {DAC1, &RCC->APB1ENR, RCC_APB1ENR_DACEN}
};

// Get index of dac res
static inline int dac_res_idx(const DAC_TypeDef *reg) {
    for(int i = 0; i < (int)DAC_RES_COUNT; i++) {
        if(dac_res[i].reg == reg) return i;
    }
    return -1;
}

#ifdef __cplusplus
}
#endif


#endif /* BMML_RES_DAC_H */