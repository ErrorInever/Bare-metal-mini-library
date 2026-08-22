#include "spi.h"
#include "bmml_dma_shared.h"
#include "bmml_res_spi.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>

#define SPI_COUNT 3

// for DMA dummy
static uint8_t dma_dummy_tx = 0xFF;
static volatile uint8_t dma_dummy_rx;

static void spi_dma_tx_isr(void *ctx);
static void spi_dma_rx_isr(void *ctx);


static spi_t spi_pool[SPI_COUNT];
static bool spi_taken[SPI_COUNT];

static int spi_slot(const SPI_TypeDef *reg) {
    if(reg == SPI1) return 0;
    if(reg == SPI2) return 1;
    if(reg == SPI3) return 2;
    return -1;
}

bmml_status_t spi_acquire(SPI_TypeDef *reg, spi_baudrate_t br, spi_callback_t cb) {
    if(reg == NULL) return BMML_INVALID_ARG;
    if(br < 0 && br > 7) return BMML_INVALID_ARG;

    int slot = spi_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(spi_taken[slot]) return BMML_BUSY;

    int idx = spi_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    spi_taken[slot] = true;
    spi_t *spi = &spi_pool[slot];
    spi->res = &spi_res[idx];
    spi->callback = cb;

    // RCC
    *spi->res->rcc_reg = spi->res->rcc_mask;

    // Reset and configure SPI
    spi_reset(reg);
    spi->res->reg->CR1 |= (SPI_CR1_MSTR                         // We are master, MSTR = 1
                        | ((uint32_t)br << SPI_CR1_BR_Pos)      // Freq devider
                        | SPI_CR1_SSM                           // Manual control CS, SSM = 1
                        | SPI_CR1_SSI);                         // Need set to 1 for manual control SSM
    
    // Enable IT 
    // spi->res->reg->CR2 |= SPI_CR2_TXEIE | SPI_CR2_RXNEIE;
    // Enable NVIC
    NVIC_EnableIRQ(spi->res->spi_irq);
    NVIC_SetPriority(spi->res->spi_irq, 5);

    spi_enable(reg);

    return BMML_OK;
}

bmml_status_t spi_it_transmit(spi_t *spi, spi_transaction_t *tr) {
    if(spi == NULL) return BMML_INVALID_ARG;
    if(tr == NULL) return BMML_INVALID_ARG;
    if(spi->busy) return BMML_BUSY;
    if (tr->tx_len == 0 && tr->rx_len == 0) return BMML_INVALID_ARG;

    // Init context
    spi->busy = true;
    spi->current_ctx = tr;
    spi->p_tx_ptr = (uint8_t *)tr->tx_buff;
    spi->p_rx_ptr = tr->rx_buff;

    uint32_t cr2mask = 0;

    // TX mode only (we send data and read dummy bytes)
    if(tr->rx_buff == NULL) {
        spi->tx_cnt = tr->tx_len;
        spi->rx_cnt = tr->tx_len;
        cr2mask = (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    }
    // RX mode only (send dummy bytes for reading)
    else if(tr->tx_buff == NULL) {
        spi->rx_cnt = tr->rx_len;
        spi->tx_cnt = tr->rx_len;
        cr2mask = (SPI_CR2_TXEIE | SPI_CR2_ERRIE); // enable only TXEIE for sync first byte
    }
    // Full duplex mode
    else {
        spi->tx_cnt = tr->tx_len;
        spi->rx_cnt = tr->rx_len;
        cr2mask = (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    }

    // reset old flags
    spi_reset_flags(spi->res->reg);

    spi->res->reg->CR2 = cr2mask;

    return BMML_OK;
}

bmml_status_t spi_dma_transmit(spi_t *spi, spi_transaction_t *tr) {
    if(spi == NULL) return BMML_INVALID_ARG;
    if(tr == NULL) return BMML_INVALID_ARG;
    if(spi->busy) return BMML_BUSY;
    if (tr->tx_len == 0 && tr->rx_len == 0) return BMML_INVALID_ARG;

    // DMA TX acquired
    if(tr->tx_len > 0 && !spi->dma_tx_acquired) {
        int idx = spi_res_idx(spi->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;
        int dma_idx = dma_res_idx(spi_res[idx].dma);
        if(dma_idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(spi_res[idx].dma, spi_res[idx].dma_tx_stream, spi_dma_tx_isr, spi, &dma_stream);
        if(st != BMML_OK) return BMML_INVALID_ARG;

        spi->dma_tx_acquired = true;
        spi->dma_tx.res = &dma_res[idx];
        spi->dma_tx.stream = dma_stream;
        spi->dma_tx.num_stream = spi_res[idx].dma_tx_stream;
        spi->dma_tx.channel = spi_res[idx].dma_tx_channel;

        // Enable DMA RCC
        *spi->dma_tx.res->rcc_reg |= spi->dma_tx.res->rcc_mask;
        // Disable stream for configure 
        dma_disable_stream(spi->dma_tx.stream);
    }
    // DMA RX acquired
    if(tr->rx_len > 0 && !spi->dma_rx_acquired) {
        int idx = spi_res_idx(spi->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;
        int dma_idx = dma_res_idx(spi_res[idx].dma);
        if(dma_idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(spi_res[idx].dma, spi_res[idx].dma_rx_stream, spi_dma_rx_isr, spi, &dma_stream);
        if(st != BMML_OK) return BMML_INVALID_ARG;

        // Enable DMA RCC
        *spi->dma_rx.res->rcc_reg |= spi->dma_rx.res->rcc_mask;
        // Disable stream for configure 
        dma_disable_stream(spi->dma_rx.stream);
    }

    spi->busy = true;
    spi->ctx = *tr;
    spi->current_ctx = tr;
    
    // len of transaction
    uint16_t total_len = (tr->tx_len > tr->rx_len) ? tr->tx_len : tr->rx_len;
    spi->dma_tx.stream->NDTR = total_len;
    spi->dma_rx.stream->NDTR = total_len;

    uint32_t tx_cr = 0U;
    uint32_t rx_cr = 0U;

    // Direction
    tx_cr |= DMA_SxCR_DIR_0; // Memory to peripheral
    rx_cr |= 0U;             // Peripheral to memory

    // Select channels
    tx_cr |= (spi->dma_tx.channel << DMA_SxCR_CHSEL_Pos);
    rx_cr |= (spi->dma_rx.channel << DMA_SxCR_CHSEL_Pos);

    // Static PAR addrs
    spi->dma_tx.stream->PAR = (uint32_t)&spi->res->reg->DR;
    spi->dma_rx.stream->PAR = (uint32_t)&spi->res->reg->DR;

    // TX stream
    if(tr->tx_buff != NULL) {
        spi->dma_tx.stream->M0AR = (uint32_t)tr->tx_buff; 
        tx_cr |= DMA_SxCR_MINC; // increment memory (buffer) 
    } else {  // if we only read we must to send a dummy bites
        spi->dma_rx.stream->M0AR = (uint32_t)&dma_dummy_tx;
        // MINC disable, DMA will cyclically read the same byte, 0xFF
    }
    // RX stream
    if(tr->rx_buff != NULL) { // if read
        spi->dma_rx.stream->M0AR = (uint32_t)tr->rx_buff;
        rx_cr |= DMA_SxCR_MINC; // increment memory (buffer)
    } else { // if we only send we must to read a dummy bites
        spi->dma_rx.stream->M0AR = (uint32_t)&dma_dummy_rx;
        // MINC disable, DMA will accumulate all incoming bytes into dummy_rx
    }

    // Update CR reg
    spi->dma_tx.stream->CR = tx_cr;
    spi->dma_rx.stream->CR = rx_cr;

    // Enable streams
    spi->dma_tx.stream->CR |= DMA_SxCR_EN;
    spi->dma_rx.stream->CR |= DMA_SxCR_EN;
    // Enable Errors
    spi->res->reg->CR2 |= SPI_CR2_ERRIE;
    // Enable DMA
    spi->res->reg->CR2 |=(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    return BMML_OK;
}