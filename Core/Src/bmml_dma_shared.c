#include "bmml_dma_shared.h"
#include "stm32f446xx.h"

static bool dma1_stream_taken[DMA_STREAM_COUNT];
static bool dma2_stream_taken[DMA_STREAM_COUNT];


bool bmml_register_dma_stream_handler(DMA_TypeDef *reg, uint8_t stream) {
    if(reg == DMA1 && !dma1_stream_taken[stream]) {
        dma1_stream_taken[stream] = true;
        return true;
    }
    if(reg == DMA2 && !dma2_stream_taken[stream]) {
        dma2_stream_taken[stream] = true;
        return true;
    }
    return false;
}

bool bmml_unregister_dma_stream_handler(DMA_TypeDef *reg, uint8_t stream) {
    if(reg == DMA1 && !dma1_stream_taken[stream]) {
        dma1_stream_taken[stream] = false;
        return true;
    }
    if(reg == DMA2 && !dma2_stream_taken[stream]) {
        dma2_stream_taken[stream] = false;
        return true;
    }
    return false;
}