#include "bmml_dma_shared.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stddef.h>

static bool dma1_stream_taken[DMA_STREAM_COUNT];
static bool dma2_stream_taken[DMA_STREAM_COUNT];

static DMA_Stream_TypeDef* const dma1_streams[] = {
    DMA1_Stream0,DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, 
    DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7
};

static DMA_Stream_TypeDef* const dma2_streams[] = {
    DMA2_Stream0,DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, 
    DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
};

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
        if(!dma1_stream_taken[stream]) return BMML_INVALID_ARG;
        dma1_stream_taken[stream] = false;
        return BMML_OK;
    }
    if(dma == DMA2) {
        if(!dma2_stream_taken[stream]) return BMML_INVALID_ARG;
        dma2_stream_taken[stream] = false;
        return BMML_OK;
    }
    return BMML_INVALID_ARG;
}