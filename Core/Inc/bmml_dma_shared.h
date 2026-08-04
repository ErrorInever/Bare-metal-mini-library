#ifndef BMML_DMA_SHARED_H
#define BMML_DMA_SHARED_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx.h"

#define DMA_STREAM_COUNT 8

typedef struct {
    DMA_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
} dma_res_t;

static const DMA_Stream_TypeDef* dma1_streams[] = {
    DMA1_Stream0,DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, 
    DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7
};

static const DMA_Stream_TypeDef* dma2_streams[] = {
    DMA2_Stream0,DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, 
    DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
};

static const dma_res_t dma_res[] = {
    {DMA1, &RCC->AHB1ENR,RCC_AHB1ENR_DMA1EN}, 
    {DMA2, &RCC->AHB1ENR,RCC_AHB1ENR_DMA2EN}, 
};

bool bmml_register_dma_stream_handler(DMA_TypeDef *reg, uint8_t stream);
bool bmml_unregister_dma_stream_handler(DMA_TypeDef *reg, uint8_t stream);


#ifdef __cplusplus
}
#endif

#endif /*BMML_DMA_SHARED_H*/