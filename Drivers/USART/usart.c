#include "usart.h"
#include "bmml_dma_shared.h"
#include "bmml_ring_buffer.h"
#include "bmml_status.h"
#include "bmml_irq_shared.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include "bmml_utilities.h"
#include <stdarg.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#ifndef USART_PRINTF_BUF_SIZE
#define USART_PRINTF_BUF_SIZE 96
#endif

#ifndef USART_COUNT
#define USART_COUNT 6
#endif

#ifndef RING_BUF_SIZE
#define RING_BUF_SIZE 128
#endif

static void usart_dma_tx_isr(void *ctx);
static void usart_dma_rx_isr(void *ctx);

static uint8_t rx_storage[USART_COUNT][RING_BUF_SIZE];    // ringbuffer recive storage
static RingBuffer_t rx_handler[USART_COUNT];              // ringbuffer recive struct

static uint8_t tx_storage[USART_COUNT][RING_BUF_SIZE];    // ringbuffer recive storage
static RingBuffer_t tx_handler[USART_COUNT]; 

static usart_t usart_pool[USART_COUNT];
static bool usart_taken[USART_COUNT];

static int usart_slot(const USART_TypeDef *reg) {
    if(reg == USART1) return 0;
    if(reg == USART2) return 1;
    if(reg == USART3) return 2;
    if(reg == UART4)  return 3;
    if(reg == UART5)  return 4;
    if(reg == USART6) return 5;
    return -1;
}

static inline uint32_t dma_htif_mask(uint32_t tcif_mask) { return tcif_mask >> 1; }

static void usart_notify(usart_t *u, usart_event_t ev) {
    if(u->callback != NULL) u->callback(u, ev);
}

static void usart_dma_stop(dma_t *d) {
    (void)dma_disable_stream(d->stream);       /* bounded wait, safe in ISR */
    dma_clear_stream_flags(d->res->reg, d->num_stream);
}

/* Acquire the DMA stream of one direction (lazy, kept until usart_release) */
static bmml_status_t usart_dma_acquire_dir(usart_t *u, bool tx) {
    bool *acquired = tx ? &u->dma_tx_acquired : &u->dma_rx_acquired;
    if(*acquired) return BMML_OK;

    dma_t *d = tx ? &u->dma_tx : &u->dma_rx;
    uint8_t  stream  = tx ? u->res->dma_tx_stream  : u->res->dma_rx_stream;
    uint32_t channel = tx ? u->res->dma_tx_channel : u->res->dma_rx_channel;

    int dma_idx = dma_res_idx(u->res->dma);
    if(dma_idx < 0) return BMML_INVALID_ARG;

    DMA_Stream_TypeDef *s;
    bmml_status_t st = bmml_dma_acquire_stream(u->res->dma, stream,
                           tx ? usart_dma_tx_isr : usart_dma_rx_isr, u, &s);
    if(st != BMML_OK) return st;

    *acquired     = true;
    d->res        = &dma_res[dma_idx];
    d->stream     = s;
    d->num_stream = stream;
    d->channel    = channel;
    *d->res->rcc_reg |= d->res->rcc_mask;
    return BMML_OK;
}

/* Move everything the RX DMA wrote since the last call into rx_buffer.
   Called from the IDLE, DMA half-transfer and DMA transfer-complete paths, which can preempt each
   other (different IRQs/priorities), so the whole thing is a critical section. */
static void usart_dma_rx_drain(usart_t *u) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t head = (uint16_t)(u->dma_rx_len - (uint16_t)u->dma_rx.stream->NDTR);
    if(head >= u->dma_rx_len) head = 0U;       /* NDTR was just reloaded */

    while(u->dma_rx_tail != head) {
        ring_buffer_push(u->rx_buffer, &u->dma_rx_buf[u->dma_rx_tail]);   /* a full buffer counts in ->dropped */
        if(++u->dma_rx_tail >= u->dma_rx_len) u->dma_rx_tail = 0U;
    }
    __set_PRIMASK(primask);
}

static bmml_status_t usart_it_enqueue(usart_t *u, const uint8_t *data, size_t len) {
    if(len == 0U) return BMML_OK;
    USART_TypeDef *r = u->res->reg;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if(r->CR3 & USART_CR3_DMAT) { __set_PRIMASK(primask); return BMML_BUSY; }

    size_t free_space = (u->tx_buffer->size - 1U) - ring_buffer_count(u->tx_buffer);
    if(len > free_space)        { __set_PRIMASK(primask); return BMML_BUSY; }   /* never truncate silently */

    for(size_t i = 0; i < len; i++) ring_buffer_push(u->tx_buffer, &data[i]);
    u->busy = true;
    r->CR1 |= USART_CR1_TXEIE;                 /* TXE is 1 when idle, the ISR fires at once */

    __set_PRIMASK(primask);
    return BMML_OK;
}

bmml_status_t usart_acquire(USART_TypeDef *reg, uint32_t br, usart_callback_t cb, usart_t **out) {
    if(out) *out = NULL;
    if(reg == NULL || br == 0U) return BMML_INVALID_ARG;

    int slot = usart_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(usart_taken[slot]) return BMML_BUSY;

    int idx = usart_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    /* Validate the baud rate BEFORE claiming the slot */
    const bool apb2 = (reg == USART1 || reg == USART6);
    const uint32_t clk = apb2 ? get_apb2_clock_hz() : get_apb1_clock_hz();
    const uint32_t brr = (clk + br / 2U) / br;          /* oversampling by 16, rounded to nearest */
    if(brr < 16U || brr > 0xFFFFU) return BMML_INVALID_ARG;

    usart_taken[slot] = true;
    usart_t *u = &usart_pool[slot];
    *u = (usart_t){0};
    u->res       = &usart_res[idx];
    u->callback  = cb;
    u->rx_buffer = &rx_handler[slot];
    u->tx_buffer = &tx_handler[slot];
    ring_buffer_init(u->rx_buffer, rx_storage[slot], RING_BUF_SIZE, sizeof(uint8_t));
    ring_buffer_init(u->tx_buffer, tx_storage[slot], RING_BUF_SIZE, sizeof(uint8_t));

    *u->res->rcc_reg |= u->res->rcc_mask;
    (void)*u->res->rcc_reg;                              /* RCC write-to-effect delay */

    reg->CR1 = 0U;                                       /* clean state, UE = 0 */
    reg->CR2 = 0U;
    reg->CR3 = 0U;
    reg->BRR = brr;
    reg->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;

    NVIC_SetPriority(u->res->usart_irq, BMML_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(u->res->usart_irq);
    NVIC_EnableIRQ(u->res->usart_irq);

    usart_enable(reg);
    (void)reg->SR; (void)reg->DR;                        /* drop stale flags */

    if(out) *out = u;
    return BMML_OK;
}

bmml_status_t usart_release(usart_t *u) {
    if(u == NULL || u->res == NULL) return BMML_INVALID_ARG;
    if(u->busy) return BMML_BUSY;                        /* wait for TX_DONE, do not cut the tail */

    USART_TypeDef *r = u->res->reg;
    int slot = usart_slot(r);
    if(slot < 0 || !usart_taken[slot]) return BMML_INVALID_ARG;

    NVIC_DisableIRQ(u->res->usart_irq);
    r->CR1 = 0U;                                         /* also clears UE and all IRQ enables */
    r->CR3 = 0U;

    if(u->dma_tx_acquired) {
        usart_dma_stop(&u->dma_tx);
        bmml_dma_release_stream(u->dma_tx.res->reg, u->dma_tx.num_stream);
    }
    if(u->dma_rx_acquired) {
        usart_dma_stop(&u->dma_rx);
        bmml_dma_release_stream(u->dma_rx.res->reg, u->dma_rx.num_stream);
    }

    *u->res->rcc_reg &= ~u->res->rcc_mask;

    usart_taken[slot] = false;
    usart_pool[slot]  = (usart_t){0};
    return BMML_OK;
}



bmml_status_t usart_it_transmit(usart_t *u, const char *data) {
    if(u == NULL || u->res == NULL || data == NULL) return BMML_INVALID_ARG;
    return usart_it_enqueue(u, (const uint8_t *)data, strlen(data));
}


bmml_status_t usart_dma_transmit(usart_t *u, const char *data, uint16_t size) {
    if(u == NULL || u->res == NULL) return BMML_INVALID_ARG;
    if(data == NULL || size == 0U) return BMML_INVALID_ARG;
    if(u->busy) return BMML_BUSY;

    bmml_status_t st = usart_dma_acquire_dir(u, true);
    if(st != BMML_OK) return st;

    if(dma_disable_stream(u->dma_tx.stream) != BMML_OK) return BMML_TIMEOUT;
    dma_clear_stream_flags(u->dma_tx.res->reg, u->dma_tx.num_stream);

    USART_TypeDef *r = u->res->reg;
    u->busy     = true;
    u->status   = BMML_OK;
    u->hw_error = 0U;

    u->dma_tx.stream->PAR  = (uint32_t)&r->DR;
    u->dma_tx.stream->M0AR = (uint32_t)data;
    u->dma_tx.stream->NDTR = size;
    u->dma_tx.stream->CR   = (u->dma_tx.channel << DMA_SxCR_CHSEL_Pos)
                           | DMA_SxCR_MINC
                           | DMA_SxCR_DIR_0                    /* memory -> peripheral */
                           | DMA_SxCR_TCIE | DMA_SxCR_TEIE;

    r->SR = ~USART_SR_TC;                                      /* drop a stale TC */
    u->dma_tx.stream->CR |= DMA_SxCR_EN;
    r->CR3 |= USART_CR3_DMAT;
    return BMML_OK;
}

bmml_status_t usart_dma_start_rx(usart_t *usart, uint8_t *buf, uint16_t buf_len) {
    if(usart == NULL || usart->res == NULL) return BMML_INVALID_ARG;
    if(buf == NULL || buf_len == 0U) return BMML_INVALID_ARG;
    if(usart->dma_rx_active) return BMML_BUSY;          /* stop it first with usart_dma_stop_rx() */

    bmml_status_t st = usart_dma_acquire_dir(usart, false);
    if(st != BMML_OK) return st;

    /* The stream must be off before PAR/M0AR/NDTR/CR are touched */
    if(dma_disable_stream(usart->dma_rx.stream) != BMML_OK) return BMML_TIMEOUT;
    dma_clear_stream_flags(usart->dma_rx.res->reg, usart->dma_rx.num_stream);

    USART_TypeDef *r = usart->res->reg;

    /* IT reception off first, so the RXNE ISR and the DMA never compete for DR */
    r->CR1 &= ~USART_CR1_RXNEIE;

    /* State used by usart_dma_rx_drain(): must be valid BEFORE the stream runs */
    usart->dma_rx_buf  = buf;
    usart->dma_rx_len  = buf_len;
    usart->dma_rx_tail = 0U;
    usart->dma_rx_active = true;

    usart->dma_rx.stream->PAR  = (uint32_t)&r->DR;
    usart->dma_rx.stream->M0AR = (uint32_t)buf;
    usart->dma_rx.stream->NDTR = buf_len;

    uint32_t cr = (usart->dma_rx.channel << DMA_SxCR_CHSEL_Pos)
                | DMA_SxCR_MINC                         /* memory increment, peripheral fixed, P2M */
                | DMA_SxCR_CIRC                         /* circular */
                | DMA_SxCR_HTIE | DMA_SxCR_TCIE         /* drain on half / full buffer */
                | DMA_SxCR_TEIE;
    usart->dma_rx.stream->CR = cr;
    usart->dma_rx.stream->CR |= DMA_SxCR_EN;

    r->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;
    r->CR3 |= USART_CR3_DMAR;                           /* DMA requests from RXNE; IDLE drains the tail */

    return BMML_OK;
}

bmml_status_t usart_printf(usart_t *u, const char *format, ...) {
    if(u == NULL || u->res == NULL || format == NULL) return BMML_INVALID_ARG;

    char buf[USART_PRINTF_BUF_SIZE];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if(n < 0)  return BMML_ERROR;
    if(n == 0) return BMML_OK;

    const bool truncated = ((size_t)n >= sizeof(buf));
    const size_t len = truncated ? sizeof(buf) - 1U : (size_t)n;

    bmml_status_t st = usart_it_enqueue(u, (const uint8_t *)buf, len);   /* BUSY / DMA check is inside */
    if(st != BMML_OK) return st;
    return truncated ? BMML_ERROR : BMML_OK;                             /* ERROR = sent, but cut off */
}

bmml_status_t usart_dma_stop_rx(usart_t *u) {
    if(u == NULL || u->res == NULL) return BMML_INVALID_ARG;
    if(!u->dma_rx_active) return BMML_OK;

    USART_TypeDef *r = u->res->reg;
    r->CR3 &= ~USART_CR3_DMAR;
    (void)dma_disable_stream(u->dma_rx.stream);
    usart_dma_rx_drain(u);                     /* NDTR is frozen now, take the rest */
    dma_clear_stream_flags(u->dma_rx.res->reg, u->dma_rx.num_stream);
    u->dma_rx_active = false;
    r->CR1 |= USART_CR1_RXNEIE;                /* back to interrupt reception */
    return BMML_OK;
}

size_t usart_read(usart_t *u, uint8_t *dst, size_t max) {
    if(u == NULL || u->rx_buffer == NULL || dst == NULL) return 0U;
    size_t n = 0U;
    while(n < max && ring_buffer_pop(u->rx_buffer, &dst[n])) n++;
    return n;
}

static void USARTx_IRQHandler(int slot) {
    usart_t *u = &usart_pool[slot];
    USART_TypeDef *r = u->res->reg;
    uint32_t sr  = r->SR;
    uint32_t cr1 = r->CR1;
    bool dr_read = false;

    /* 1) line errors. SR read + DR read clears them. In DMA mode this read can take one byte
          away from the DMA, but we are already in an error state. */
    uint32_t err = sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);
    if(err) {
        (void)r->DR;
        dr_read = true;
        sr &= ~USART_SR_RXNE;                  /* that byte is gone: do not read DR again below */
        u->hw_error = err;
        usart_notify(u, USART_EVENT_ERROR);
    }

    /* 2) received byte (IT mode; RXNEIE is off while the DMA is active) */
    if((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
        uint8_t ch = usart_receive_byte(r);
        dr_read = true;
        ring_buffer_push(u->rx_buffer, &ch);   /* when full: rx_buffer->dropped++ */
    }

    /* 3) TX register empty: next byte from the queue */
    if((sr & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE)) {
        uint8_t b;
        if(ring_buffer_pop(u->tx_buffer, &b)) {
            r->DR = b;
        } else {
            r->CR1 &= ~USART_CR1_TXEIE;
            r->CR1 |= USART_CR1_TCIE;          /* now wait until the last byte has really left */
        }
    }

    /* 4) transmission complete (IT queue drained or DMA finished) */
    if((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE)) {
        r->CR1 &= ~USART_CR1_TCIE;
        r->SR = ~USART_SR_TC;
        if(is_empty(u->tx_buffer)) {           /* new data may have been queued in the meantime */
            u->status = BMML_OK;
            u->busy   = false;
            usart_notify(u, USART_EVENT_TX_DONE);
        }
    }

    /* 5) IDLE: the line was quiet for one frame -> a burst of data ended */
    if((sr & USART_SR_IDLE) && (cr1 & USART_CR1_IDLEIE)) {
        if(!dr_read) (void)r->DR;              /* IDLE is cleared by SR read (above) + DR read */
        if(u->dma_rx_active) usart_dma_rx_drain(u);
        if(!is_empty(u->rx_buffer)) usart_notify(u, USART_EVENT_RX);
    }
}

/* ---- DMA error handling: each direction is aborted on its own ---- */

static void usart_dma_tx_abort(usart_t *u, uint32_t hw_error) {
    USART_TypeDef *r = u->res->reg;
    r->CR3 &= ~USART_CR3_DMAT;
    r->CR1 &= ~USART_CR1_TCIE;
    usart_dma_stop(&u->dma_tx);
    u->status   = BMML_ERROR;
    u->hw_error = hw_error;
    u->busy     = false;
    usart_notify(u, USART_EVENT_ERROR);
}

static void usart_dma_rx_abort(usart_t *u, uint32_t hw_error) {
    USART_TypeDef *r = u->res->reg;
    r->CR3 &= ~USART_CR3_DMAR;
    usart_dma_rx_drain(u);                     /* keep what was already received */
    usart_dma_stop(&u->dma_rx);
    u->dma_rx_active = false;
    r->CR1 |= USART_CR1_RXNEIE;                /* fall back to interrupt reception */
    u->status   = BMML_ERROR;
    u->hw_error = hw_error;
    usart_notify(u, USART_EVENT_ERROR);
}

static void usart_dma_tx_isr(void *ctx) {
    usart_t *u = (usart_t *)ctx;
    const uint32_t teif = dma_teif_mask(u->res->tx_tcif_mask);

    if(dma_check_flag(&u->dma_tx, teif, true)) {
        usart_dma_tx_abort(u, teif);
        return;
    }
    if(dma_check_flag(&u->dma_tx, u->res->tx_tcif_mask, true)) {
        /* The last byte went into DR but is not on the wire yet: USART TC finishes the job */
        u->res->reg->CR3 &= ~USART_CR3_DMAT;
        u->res->reg->CR1 |= USART_CR1_TCIE;
    }
}

static void usart_dma_rx_isr(void *ctx) {
    usart_t *u = (usart_t *)ctx;
    const uint32_t teif = dma_teif_mask(u->res->rx_tcif_mask);

    if(dma_check_flag(&u->dma_rx, teif, true)) {
        usart_dma_rx_abort(u, teif);
        return;
    }
    /* No short-circuit: both flags must be checked and cleared */
    bool ht = dma_check_flag(&u->dma_rx, dma_htif_mask(u->res->rx_tcif_mask), true);
    bool tc = dma_check_flag(&u->dma_rx, u->res->rx_tcif_mask, true);
    if(ht || tc) {
        usart_dma_rx_drain(u);
        if(!is_empty(u->rx_buffer)) usart_notify(u, USART_EVENT_RX);
    }
}

// IRQ Handlers
void USART1_IRQHandler(void) {
    if(!usart_taken[0]) return;
    USARTx_IRQHandler(0);
}

void USART2_IRQHandler(void) {
    if(!usart_taken[1]) return;
    USARTx_IRQHandler(1);
}

void USART3_IRQHandler(void) {
    if(!usart_taken[2]) return;
    USARTx_IRQHandler(2);
}

void UART4_IRQHandler(void) {
    if(!usart_taken[3]) return;
    USARTx_IRQHandler(3);
}

void UART5_IRQHandler(void) {
    if(!usart_taken[4]) return;
    USARTx_IRQHandler(4);
}

void USART6_IRQHandler(void) {
    if(!usart_taken[5]) return;
    USARTx_IRQHandler(5);
}