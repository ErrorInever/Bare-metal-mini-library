#ifndef BMML_DMA_SHARED_H
#define BMML_DMA_SHARED_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx.h"
#include "bmml_status.h"

#define DMA_STREAM_COUNT 8

// callback for DMA with context
typedef void (*dma_stream_cb_t)(void *ctx);

typedef struct {
    DMA_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
} dma_res_t;

typedef struct {
    const dma_res_t *res;
    DMA_Stream_TypeDef *stream;
    uint8_t num_stream;
    uint32_t channel;
} dma_t;

static const dma_res_t dma_res[] = {
    {DMA1, &RCC->AHB1ENR,RCC_AHB1ENR_DMA1EN}, 
    {DMA2, &RCC->AHB1ENR,RCC_AHB1ENR_DMA2EN}, 
};

static inline int dma_res_idx(const DMA_TypeDef *reg) {
    for (int i = 0; i < 2; i++) if (dma_res[i].reg == reg) return i;
    return -1;
}

static inline bmml_status_t dma_disable_stream(DMA_Stream_TypeDef *stream) {
    stream->CR &= ~DMA_SxCR_EN;
    uint32_t timeout = 10000;
    while(stream->CR & DMA_SxCR_EN)
        if(--timeout == 0) return BMML_TIMEOUT;
    return BMML_OK;
}

static inline void dma_clear_stream_flags(DMA_TypeDef *dma, uint8_t stream) {
    static const uint32_t flag_mask[4] = {
        DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0,
        DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1,
        DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2,
        DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3,
    };
    uint32_t mask = flag_mask[stream % 4];
    if(stream < 4) dma->LIFCR = mask;
    else dma->HIFCR = mask;
}

bmml_status_t bmml_dma_acquire_stream(DMA_TypeDef *dma, uint8_t stream, dma_stream_cb_t cb, void *ctx, DMA_Stream_TypeDef **out_stream);
bmml_status_t bmml_dma_release_stream(DMA_TypeDef *dma, uint8_t stream);


#ifdef __cplusplus
}
#endif

#endif /*BMML_DMA_SHARED_H*/