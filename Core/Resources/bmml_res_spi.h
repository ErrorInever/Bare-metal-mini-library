#ifndef BMML_RES_SPI_H
#define BMML_RES_SPI_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"

#define SPI_RES_COUNT 3

typedef struct {
    SPI_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
    IRQn_Type spi_irq;
    uint8_t  dma_tx_stream;
    uint32_t dma_tx_channel;
    uint8_t  dma_rx_stream;
    uint32_t dma_rx_channel;
    uint32_t tx_tcif_mask;
    uint32_t rx_tcif_mask;
} spi_res_t;

static const spi_res_t spi_res[] = {
    // SPI1 (APB2, DMA2, channel 3)
    { SPI1, &RCC->APB2ENR, RCC_APB2ENR_SPI1EN, SPI1_IRQn,
      3, 3,   0, 3,                       // TX: stream3 ch3   RX: stream0 ch3
      DMA_LISR_TCIF3, DMA_LISR_TCIF0 },

    // SPI2 (APB1, DMA1, channel 0)
    { SPI2, &RCC->APB1ENR, RCC_APB1ENR_SPI2EN, SPI2_IRQn,
      4, 0,   3, 0,                       // TX: stream4 ch0   RX: stream3 ch0
      DMA_HISR_TCIF4, DMA_LISR_TCIF3 },

    // SPI3 (APB1, DMA1, channel 0)
    { SPI3, &RCC->APB1ENR, RCC_APB1ENR_SPI3EN, SPI3_IRQn,
      5, 0,   0, 0,                       // TX: stream5 ch0   RX: stream0 ch0
      DMA_HISR_TCIF5, DMA_LISR_TCIF0 },
};

static inline int spi_res_idx(const SPI_TypeDef *reg) {
    for (int i = 0; i < SPI_RES_COUNT; i++) {
        if (spi_res[i].reg == reg) return i;
    }
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif /*BMML_RES_SPI_H*/