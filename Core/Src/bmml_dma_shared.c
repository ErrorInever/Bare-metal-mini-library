#include "bmml_dma_shared.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stddef.h>

static bool dma1_stream_taken[DMA_STREAM_COUNT];
static bool dma2_stream_taken[DMA_STREAM_COUNT];


bmml_status_t bmml_dma_acquire_stream(DMA_TypeDef *dma, uint8_t stream, DMA_Stream_TypeDef **out_stream) {
    if(out_stream) *out_stream = NULL;
    if(stream >= DMA_STREAM_COUNT) return BMML_INVALID_ARG;

    if(dma == DMA1) {
        if(dma1_stream_taken[stream]) return BMML_BUSY;
        dma1_stream_taken[stream] = true;
        if(out_stream) *out_stream = dma1_streams[stream];
        return BMML_OK;
    }
    if(dma == DMA2) {
        if(dma2_stream_taken[stream]) return BMML_BUSY;
        dma2_stream_taken[stream] = true;
        if(out_stream) *out_stream = dma2_streams[stream];
        return BMML_OK;
    }
    return BMML_INVALID_ARG;
}

bmml_status_t bmml_dma_release_stream(DMA_TypeDef *dma, uint8_t stream) {
    if(stream >= DMA_STREAM_COUNT) return BMML_INVALID_ARG;
    
    if(dma == DMA1) {
        dma1_stream_taken[stream] = false;
        return BMML_OK;
    }
    if(dma == DMA2) {
        dma2_stream_taken[stream] = false;
        return BMML_OK;
    }
    return BMML_INVALID_ARG;
}