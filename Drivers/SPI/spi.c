#include "spi.h"
#include "bmml_dma_shared.h"
#include "bmml_res_spi.h"
#include "bmml_status.h"
#include "bmml_irq_shared.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>

#define SPI_COUNT 4

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
    if(reg == SPI4) return 3;
    return -1;
}

/* Transfer length, or 0 if the transaction is invalid */
static uint16_t spi_xfer_len(const spi_transaction_t *tr) {
    if(tr->tx_buff == NULL && tr->rx_buff == NULL) return 0;
    if(tr->tx_buff != NULL && tr->rx_buff != NULL)
        return (tr->tx_len == tr->rx_len) ? tr->tx_len : 0;   /* full duplex: lengths must match */
    return (tr->tx_buff != NULL) ? tr->tx_len : tr->rx_len;
}

static void spi_wait_idle(SPI_TypeDef *reg) {
    uint32_t t = 10000U;
    while((reg->SR & SPI_SR_BSY) && --t);
}

/* Single exit point of every transfer */
static void spi_complete(spi_t *spi, bmml_status_t status, uint32_t hw_error) {
    spi->res->reg->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE |
                            SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    spi->status   = status;
    spi->hw_error = hw_error;
    spi->busy     = false;

    /* per-transaction callback has priority over the driver-level one */
    spi_callback_t cb = (spi->ctx.callback != NULL) ? spi->ctx.callback : spi->callback;
    if(cb != NULL) cb(spi);
}

bmml_status_t spi_acquire(SPI_TypeDef *reg, spi_baudrate_t br, spi_callback_t cb,  spi_t **out) {
    if(reg == NULL) return BMML_INVALID_ARG;
    if(out) *out = NULL;
    if(br > SPI_BAUDRATE_DIV256 || br < SPI_BAUDRATE_DIV2) return BMML_INVALID_ARG;

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
    *spi->res->rcc_reg |= spi->res->rcc_mask;

    // Reset and configure SPI
    spi_reset(reg);
    spi->res->reg->CR1 |= (SPI_CR1_MSTR                         // We are master, MSTR = 1
                        | ((uint32_t)br << SPI_CR1_BR_Pos)      // Freq devider
                        | SPI_CR1_SSM                           // Manual control CS, SSM = 1
                        | SPI_CR1_SSI);                         // Need set to 1 for manual control SSM

    // Enable NVIC
    NVIC_SetPriority(spi->res->spi_irq, BMML_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(spi->res->spi_irq);
    NVIC_EnableIRQ(spi->res->spi_irq);

    spi_enable(reg);

    if(out) *out = spi;

    return BMML_OK;
}

bmml_status_t spi_release(spi_t *spi) {
    if(spi == NULL || spi->res == NULL) return BMML_INVALID_ARG;
    if(spi->busy) return BMML_BUSY;

    int slot = spi_slot(spi->res->reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(!spi_taken[slot]) return BMML_INVALID_ARG;

    spi_reset(spi->res->reg); // disable SPI module
    spi->res->reg->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
    NVIC_DisableIRQ(spi->res->spi_irq);


    if(spi->dma_tx_acquired) {
        if((dma_disable_stream(spi->dma_tx.stream)) != BMML_OK) return BMML_TIMEOUT;
        bmml_dma_release_stream(spi->dma_tx.res->reg, spi->dma_tx.num_stream);
        dma_clear_stream_flags(spi->dma_tx.res->reg, spi->dma_tx.num_stream);
    }
    if(spi->dma_rx_acquired) {
        if((dma_disable_stream(spi->dma_rx.stream)) != BMML_OK) return BMML_TIMEOUT;
        bmml_dma_release_stream(spi->dma_rx.res->reg, spi->dma_rx.num_stream);
        dma_clear_stream_flags(spi->dma_rx.res->reg, spi->dma_rx.num_stream);
    }

    spi_taken[slot] = false;
    spi_pool[slot] = (spi_t){0};

    return BMML_OK;
}

bmml_status_t spi_it_transmit(spi_t *spi, spi_transaction_t *tr) {
    if(spi == NULL || spi->res == NULL || tr == NULL) return BMML_INVALID_ARG;
    if(spi->busy) return BMML_BUSY;

    uint16_t len = spi_xfer_len(tr);
    if(len == 0) return BMML_INVALID_ARG;

    SPI_TypeDef *r = spi->res->reg;
    (void)r->DR; (void)r->SR;                  /* clear a stale OVR */

    spi->busy      = true;
    spi->xfer_dma  = false;
    spi->ctx       = *tr;
    spi->status    = BMML_OK;
    spi->hw_error  = 0U;
    spi->p_tx_ptr  = (uint8_t *)tr->tx_buff;   /* NULL in RX-only mode */
    spi->p_rx_ptr  = tr->rx_buff;              /* NULL in TX-only mode */
    spi->tx_cnt    = len;
    spi->rx_cnt    = len;

    uint32_t cr2 = SPI_CR2_TXEIE | SPI_CR2_ERRIE;
    if(tr->tx_buff != NULL) cr2 |= SPI_CR2_RXNEIE;   /* RX-only enables RXNEIE after the first dummy byte */
    r->CR2 |= cr2;

    return BMML_OK;
}

static bmml_status_t spi_dma_acquire_dir(spi_t *spi, bool tx) {
    bool *acquired = tx ? &spi->dma_tx_acquired : &spi->dma_rx_acquired;
    if(*acquired) return BMML_OK;

    const spi_res_t *res = spi->res;
    dma_t *d = tx ? &spi->dma_tx : &spi->dma_rx;
    uint8_t  stream  = tx ? res->dma_tx_stream  : res->dma_rx_stream;
    uint32_t channel = tx ? res->dma_tx_channel : res->dma_rx_channel;

    int dma_idx = dma_res_idx(res->dma);
    if(dma_idx < 0) return BMML_INVALID_ARG;

    DMA_Stream_TypeDef *s;
    bmml_status_t st = bmml_dma_acquire_stream(res->dma, stream,
                           tx ? spi_dma_tx_isr : spi_dma_rx_isr, spi, &s);
    if(st != BMML_OK) return st;

    *acquired     = true;
    d->res        = &dma_res[dma_idx];
    d->stream     = s;
    d->num_stream = stream;
    d->channel    = channel;
    *d->res->rcc_reg |= d->res->rcc_mask;
    return BMML_OK;
}

bmml_status_t spi_dma_transmit(spi_t *spi, spi_transaction_t *tr) {
    if(spi == NULL || spi->res == NULL || tr == NULL) return BMML_INVALID_ARG;
    if(spi->busy) return BMML_BUSY;

    uint16_t len = spi_xfer_len(tr);
    if(len == 0) return BMML_INVALID_ARG;

    bmml_status_t st = spi_dma_acquire_dir(spi, true);
    if(st != BMML_OK) return st;
    st = spi_dma_acquire_dir(spi, false);
    if(st != BMML_OK) return st;

    /* Streams must be off before they are reprogrammed; drop stale flags */
    if(dma_disable_stream(spi->dma_tx.stream) != BMML_OK) return BMML_TIMEOUT;
    if(dma_disable_stream(spi->dma_rx.stream) != BMML_OK) return BMML_TIMEOUT;
    dma_clear_stream_flags(spi->dma_tx.res->reg, spi->dma_tx.num_stream);
    dma_clear_stream_flags(spi->dma_rx.res->reg, spi->dma_rx.num_stream);

    SPI_TypeDef *r = spi->res->reg;
    (void)r->DR; (void)r->SR;                  /* clear a stale OVR */

    spi->busy          = true;
    spi->xfer_dma      = true;
    spi->ctx           = *tr;
    spi->status        = BMML_OK;
    spi->hw_error      = 0U;
    spi->tx_phase_done = false;
    spi->rx_phase_done = false;

    spi->dma_tx.stream->PAR  = (uint32_t)&r->DR;
    spi->dma_rx.stream->PAR  = (uint32_t)&r->DR;
    spi->dma_tx.stream->NDTR = len;
    spi->dma_rx.stream->NDTR = len;

    uint32_t tx_cr = (spi->dma_tx.channel << DMA_SxCR_CHSEL_Pos)
                   | DMA_SxCR_DIR_0                       /* memory -> peripheral */
                   | DMA_SxCR_TCIE | DMA_SxCR_TEIE;       /* FIX: interrupts were missing */
    uint32_t rx_cr = (spi->dma_rx.channel << DMA_SxCR_CHSEL_Pos)
                   | DMA_SxCR_TCIE | DMA_SxCR_TEIE;

    if(tr->tx_buff != NULL) {
        spi->dma_tx.stream->M0AR = (uint32_t)tr->tx_buff;
        tx_cr |= DMA_SxCR_MINC;
    } else {
        spi->dma_tx.stream->M0AR = (uint32_t)&dma_dummy_tx;   /* RX-only: clock out 0xFF */
    }
    if(tr->rx_buff != NULL) {
        spi->dma_rx.stream->M0AR = (uint32_t)tr->rx_buff;
        rx_cr |= DMA_SxCR_MINC;
    } else {
        spi->dma_rx.stream->M0AR = (uint32_t)&dma_dummy_rx;   /* TX-only: throw RX bytes away */
    }
    spi->dma_tx.stream->CR = tx_cr;
    spi->dma_rx.stream->CR = rx_cr;

    r->CR2 |= SPI_CR2_ERRIE;
    spi->dma_rx.stream->CR |= DMA_SxCR_EN;     /* RX first, so no incoming byte is lost */
    spi->dma_tx.stream->CR |= DMA_SxCR_EN;
    r->CR2 |= SPI_CR2_RXDMAEN;
    r->CR2 |= SPI_CR2_TXDMAEN;                 /* TX request starts the transfer */

    return BMML_OK;
}

static void DMA_finish(spi_t *spi) {
    spi_wait_idle(spi->res->reg);
    spi_complete(spi, BMML_OK, 0U);
}

static void DMA_tx_complete(spi_t *spi) {
    spi->res->reg->CR2 &= ~SPI_CR2_TXDMAEN;
    spi->tx_phase_done = true;
    if(spi->rx_phase_done) DMA_finish(spi);
}
static void DMA_rx_complete(spi_t *spi) {
    spi->res->reg->CR2 &= ~SPI_CR2_RXDMAEN;
    spi->rx_phase_done = true;
    if(spi->tx_phase_done) DMA_finish(spi);
}

static void spi_abort_error(spi_t *spi, uint32_t hw_error) {
    if(spi->dma_tx_acquired) {
        dma_disable_stream(spi->dma_tx.stream);
        dma_clear_stream_flags(spi->dma_tx.res->reg, spi->dma_tx.num_stream);
    }
    if(spi->dma_rx_acquired) {
        dma_disable_stream(spi->dma_rx.stream);
        dma_clear_stream_flags(spi->dma_rx.res->reg, spi->dma_rx.num_stream);
    }
    spi_complete(spi, BMML_ERROR, hw_error);
}

static void spi_dma_tx_isr(void *ctx) {
    spi_t *spi = (spi_t *)ctx;
    uint32_t teif = dma_teif_mask(spi->res->tx_tcif_mask);
    if(dma_check_flag(&spi->dma_tx, teif, false)) {
        spi_abort_error(spi, teif);           // гасит ОБА стрима + releases + callback
    } else if(dma_check_flag(&spi->dma_tx, spi->res->tx_tcif_mask, true)) {
        DMA_tx_complete(spi);
    }
}

static void spi_dma_rx_isr(void *ctx) {
    spi_t *spi = (spi_t *)ctx;
    uint32_t teif = dma_teif_mask(spi->res->rx_tcif_mask);
    if(dma_check_flag(&spi->dma_rx, teif, false)) {
        spi_abort_error(spi, teif);
    } else if(dma_check_flag(&spi->dma_rx, spi->res->rx_tcif_mask, true)) {
        DMA_rx_complete(spi);
    }
}

// IT and Errors
static void SPIx_IRQHandler(int slot) {
    spi_t *spi = &spi_pool[slot];
    if(!spi->busy) return;

    SPI_TypeDef *r = spi->res->reg;
    uint32_t sr  = r->SR;
    uint32_t cr2 = r->CR2;

    /* 1) errors: abort and leave immediately */
    uint32_t err = sr & (SPI_SR_OVR | SPI_SR_MODF);
    if((cr2 & SPI_CR2_ERRIE) && err) {
        if(err & SPI_SR_OVR)  { (void)r->DR; (void)r->SR; }   /* clear OVR: read DR, then SR */
        if(err & SPI_SR_MODF) r->CR1 &= ~SPI_CR1_SPE;         /* clear MODF: SR read + CR1 write.
                                                                 Hardware also cleared MSTR, so
                                                                 re-acquire the SPI afterwards. */
        spi_abort_error(spi, err);
        return;
    }
    if(spi->xfer_dma) return;                  /* DMA mode: only errors are handled here */

    /* 2) RXNE: read a byte (store it, or throw it away in TX-only mode) */
    if((sr & SPI_SR_RXNE) && (cr2 & SPI_CR2_RXNEIE) && spi->rx_cnt > 0) {
        uint8_t b = (uint8_t)r->DR;
        if(spi->p_rx_ptr != NULL) *spi->p_rx_ptr++ = b;
        spi->rx_cnt--;
        /* RX-only: the next dummy byte can go out now */
        if(spi->p_tx_ptr == NULL && spi->rx_cnt > 0) r->CR2 |= SPI_CR2_TXEIE;
    }

    /* 3) TXE: send the next byte */
    if((sr & SPI_SR_TXE) && (cr2 & SPI_CR2_TXEIE)) {
        if(spi->tx_cnt > 0) {
            if(spi->p_tx_ptr != NULL) {
                r->DR = *spi->p_tx_ptr++;
            } else {                           /* RX-only: dummy byte */
                r->DR = 0xFF;
                r->CR2 |= SPI_CR2_RXNEIE;
            }
            spi->tx_cnt--;
        }
        if(spi->tx_cnt == 0 || spi->p_tx_ptr == NULL) r->CR2 &= ~SPI_CR2_TXEIE;
    }

    /* 4) done */
    if(spi->tx_cnt == 0 && spi->rx_cnt == 0) {
        spi_wait_idle(r);
        spi_complete(spi, BMML_OK, 0U);
    }
}

// SPI IT handlers

void SPI1_IRQHandler(void) {
    if(!spi_taken[0]) return;
    SPIx_IRQHandler(0);
}

void SPI2_IRQHandler(void) {
    if(!spi_taken[1]) return;
    SPIx_IRQHandler(1);
}

void SPI3_IRQHandler(void) {
    if(!spi_taken[2]) return;
    SPIx_IRQHandler(2);
}

void SPI4_IRQHandler(void) {
    if(!spi_taken[3]) return;
    SPIx_IRQHandler(3);
}