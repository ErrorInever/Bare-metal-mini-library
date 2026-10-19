#ifndef BMML_USART_H
#define BMML_USART_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"

#define USART_RES_COUNT 6

typedef struct {
    USART_TypeDef *reg;
    DMA_TypeDef *dma;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
    IRQn_Type usart_irq;
    uint8_t  dma_tx_stream;
    uint32_t dma_tx_channel;
    uint8_t  dma_rx_stream;
    uint32_t dma_rx_channel;
    uint32_t tx_tcif_mask;
    uint32_t rx_tcif_mask;
} usart_res_t;

static const usart_res_t usart_res[] = {

    /* USART1 */
    {
        .reg            = USART1,
        .dma            = DMA2,
        .rcc_reg        = &RCC->APB2ENR,
        .rcc_mask       = RCC_APB2ENR_USART1EN,
        .usart_irq      = USART1_IRQn,

        .dma_tx_stream  = 7,
        .dma_tx_channel = 4,
        .dma_rx_stream  = 2,
        .dma_rx_channel = 4,

        .tx_tcif_mask   = DMA_HISR_TCIF7,
        .rx_tcif_mask   = DMA_LISR_TCIF2,
    },

    /* USART2 */
    {
        .reg            = USART2,
        .dma            = DMA1,
        .rcc_reg        = &RCC->APB1ENR,
        .rcc_mask       = RCC_APB1ENR_USART2EN,
        .usart_irq      = USART2_IRQn,

        .dma_tx_stream  = 6,
        .dma_tx_channel = 4,
        .dma_rx_stream  = 5,
        .dma_rx_channel = 4,

        .tx_tcif_mask   = DMA_HISR_TCIF6,
        .rx_tcif_mask   = DMA_HISR_TCIF5,
    },

    /* USART3 */
    {
        .reg            = USART3,
        .dma            = DMA1,
        .rcc_reg        = &RCC->APB1ENR,
        .rcc_mask       = RCC_APB1ENR_USART3EN,
        .usart_irq      = USART3_IRQn,

        .dma_tx_stream  = 3,
        .dma_tx_channel = 4,
        .dma_rx_stream  = 1,
        .dma_rx_channel = 4,

        .tx_tcif_mask   = DMA_LISR_TCIF3,
        .rx_tcif_mask   = DMA_LISR_TCIF1,
    },

    /* UART4 */
    {
        .reg            = UART4,
        .dma            = DMA1,
        .rcc_reg        = &RCC->APB1ENR,
        .rcc_mask       = RCC_APB1ENR_UART4EN,
        .usart_irq      = UART4_IRQn,

        .dma_tx_stream  = 4,
        .dma_tx_channel = 4,
        .dma_rx_stream  = 2,
        .dma_rx_channel = 4,

        .tx_tcif_mask   = DMA_HISR_TCIF4,
        .rx_tcif_mask   = DMA_LISR_TCIF2,
    },

    /* UART5 */
    {
        .reg            = UART5,
        .dma            = DMA1,
        .rcc_reg        = &RCC->APB1ENR,
        .rcc_mask       = RCC_APB1ENR_UART5EN,
        .usart_irq      = UART5_IRQn,

        .dma_tx_stream  = 7,
        .dma_tx_channel = 4,
        .dma_rx_stream  = 0,
        .dma_rx_channel = 4,

        .tx_tcif_mask   = DMA_HISR_TCIF7,
        .rx_tcif_mask   = DMA_LISR_TCIF0,
    },

    /* USART6 */
    {
        .reg            = USART6,
        .dma            = DMA2,
        .rcc_reg        = &RCC->APB2ENR,
        .rcc_mask       = RCC_APB2ENR_USART6EN,
        .usart_irq      = USART6_IRQn,

        .dma_tx_stream  = 6,
        .dma_tx_channel = 5,
        .dma_rx_stream  = 1,
        .dma_rx_channel = 5,

        .tx_tcif_mask   = DMA_HISR_TCIF6,
        .rx_tcif_mask   = DMA_LISR_TCIF1,
    },
};

// Get index of USART res
static inline int usart_res_idx(const USART_TypeDef *reg) {
    for(int i = 0; i < (int)USART_RES_COUNT; i++) {
        if(usart_res[i].reg == reg) return i;
    }
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif /*BMML_USART_H*/