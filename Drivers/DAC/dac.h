#ifndef DAC_H
#define DAC_H


#include "stm32f446xx.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bmml_res_dac.h"
#include "bmml_status.h"
#include "bmml_dma_shared.h"
#include "timer.h"


typedef struct {
    const dac_res_t *res;
    dma_t dma;
    timer_basic_t *timer;
    uint16_t *wave_buffer;      /**< Pointer to source memory buffer (RAM) containing digital wave samples. */
    uint16_t num_points;        /**< Total number of signal points/samples allocated inside the wave buffer. */
} dac_t;

#ifdef __cplusplus
}
#endif

bmml_status_t dac_dma_acquire(DAC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream, uint16_t *wave_buffer, 
    uint16_t num_points, dac_t **out);

bmml_status_t dac_dma_release(dac_t *dac);

static inline void dac_start(dac_t *dac) {
    dac->dma.stream->CR |= DMA_SxCR_EN;
}

static inline void dac_stop(dac_t *dac) {
    dac->dma.stream->CR &= ~DMA_SxCR_EN;
}

#endif /* DAC_H */