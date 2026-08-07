#include "adc.h"
#include "bmml_status.h"
#include <stddef.h>

#define ADC_COUNT 3

static adc_t adc_pool[ADC_COUNT];
static bool adc_taken[ADC_COUNT];

static int adc_slot(const ADC_TypeDef *reg) {
    if(reg == ADC1) return 0;
    if(reg == ADC2) return 1;
    if(reg == ADC3) return 2;
    return -1;
}


bmml_status_t adc_dma_acquire(ADC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream, adc_mode_t mode, uint16_t *data_buffer, adc_t **out) {
    if(out) *out = NULL;
    if(reg == NULL || dma == NULL) return BMML_INVALID_ARG;

    int slot = adc_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(adc_taken[slot]) return BMML_BUSY;

    

    return BMML_OK;
}