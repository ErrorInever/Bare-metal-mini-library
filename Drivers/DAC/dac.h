#ifndef DAC_H
#define DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx.h"
#include "bmml_res_dac.h"
#include "bmml_status.h"
#include "bmml_dma_shared.h"
#include "timer.h"

typedef struct {
    const dac_res_t *res;
    dma_t dma;
    timer_basic_t *timer;
    uint16_t *wave_buffer;      /**< 12-bit right-aligned samples (0..4095). Must outlive the DAC handle. */
    uint16_t num_points;
    uint32_t sample_rate_hz;
    bool running;
} dac_t;

/**
 * @brief Channel 1 of DAC1 driven by TIM6 TRGO through DMA1 Stream5 (circular).
 * Output pin PA4 must be configured as GPIO_MODE_ANALOG by the caller.
 * Output signal frequency = sample_rate_hz / num_points.
 * Generation does not start until dac_start().
 */
bmml_status_t dac_dma_acquire(DAC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream,
                              uint16_t *wave_buffer, uint16_t num_points,
                              uint32_t sample_rate_hz, dac_t **out);
bmml_status_t dac_dma_release(dac_t *dac);
bmml_status_t dac_start(dac_t *dac);
bmml_status_t dac_stop(dac_t *dac);
bmml_status_t dac_set_sample_rate(dac_t *dac, uint32_t sample_rate_hz);

#ifdef __cplusplus
}
#endif

#endif /* DAC_H */