#include "usart.h"
#include "bmml_dma_shared.h"
#include "bmml_ring_buffer.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include "bmml_utilities.h"
#include <stdarg.h>
#include <stdio.h>
#include <stddef.h>

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

bmml_status_t usart_acquire(USART_TypeDef *reg, uint32_t br, usart_callback_t cb, usart_t **out) {
    if(reg == NULL) return BMML_INVALID_ARG;
    if(br == 0) return BMML_INVALID_ARG;
    if(out) *out = NULL;

    int slot = usart_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(usart_taken[slot]) return BMML_BUSY;

    int idx = usart_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    usart_taken[slot] = true;
    usart_t *usart = &usart_pool[slot];
    usart->res = &usart_res[idx];
    usart->callback = cb;

    // Init ringbuffers for USART
    usart->rx_buffer = &rx_handler[slot];
    usart->tx_buffer = &tx_handler[slot];

    ring_buffer_init(usart->rx_buffer, rx_storage[slot], 
        RING_BUF_SIZE, sizeof(uint8_t));
    usart->rx_buffer = &rx_handler[slot];

    ring_buffer_init(usart->tx_buffer, tx_storage[slot], 
        RING_BUF_SIZE, sizeof(uint8_t));
    usart->tx_buffer = &tx_handler[slot];

    // RCC
    *usart->res->rcc_reg |= usart->res->rcc_mask;

    bool bus = (reg == USART1 || reg == USART6) ? true : false;
    uint32_t bus_freq;
    if(bus) {
        bus_freq = get_apb2_clock_hz();
    } else {
        bus_freq = get_apb1_clock_hz();
    }
    usart->res->reg->BRR = (bus_freq + br / 2) / br;
    usart->res->reg->CR1 |= USART_CR1_TE |                           // Enable transmitter
                            USART_CR1_RE |                           // Enable receiver
                            USART_CR1_RXNEIE;                        // Enable interrupt on receive
    usart->res->reg->CR1 &= ~USART_CR1_TXEIE;                        // Disable TXE 

    // Enable NVIC
    NVIC_EnableIRQ(usart->res->usart_irq);
    NVIC_SetPriority(usart->res->usart_irq, 5);

    usart_enable(usart->res->reg);

    *out = usart;

    return BMML_OK;
}

bmml_status_t usart_release(usart_t *usart) {
    if(usart == NULL || usart->res == NULL) return BMML_INVALID_ARG;
    if(usart->busy) return BMML_BUSY;

    int slot = usart_slot(usart->res->reg);
    if(slot < 0 || !usart_taken[slot]) return BMML_INVALID_ARG;

    usart->res->reg->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_IDLEIE);
    usart->res->reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
    NVIC_DisableIRQ(usart->res->usart_irq);
    usart_disable(usart->res->reg);

    if(usart->dma_tx_acquired) {
        dma_disable_stream(usart->dma_tx.stream);
        dma_clear_stream_flags(usart->dma_tx.res->reg, usart->dma_tx.num_stream);
        bmml_dma_release_stream(usart->dma_tx.res->reg, usart->dma_tx.num_stream);
    }
    if(usart->dma_rx_acquired) {
        dma_disable_stream(usart->dma_rx.stream);
        dma_clear_stream_flags(usart->dma_rx.res->reg, usart->dma_rx.num_stream);
        bmml_dma_release_stream(usart->dma_rx.res->reg, usart->dma_rx.num_stream);
    }

    usart_taken[slot] = false;
    usart_pool[slot] = (usart_t){0};
    return BMML_OK;
}



bmml_status_t usart_it_transmit(usart_t *usart, const char *data) {
    if(usart == NULL || usart->res == NULL) return BMML_INVALID_ARG;
    if(usart->busy) return BMML_BUSY;

    __disable_irq();
    while(*data) {
        char ch = *data++;
        ring_buffer_push(usart->tx_buffer, &ch);
    }
    __enable_irq();

    // Enable IT
    usart->busy = true;
    usart->res->reg->CR1 |= USART_CR1_TXEIE;

    return BMML_OK;
}

bmml_status_t usart_dma_transmit(usart_t *usart, const char *data, uint16_t size) {
    if(usart == NULL || usart->res == NULL) return BMML_INVALID_ARG;
    if(usart->busy) return BMML_BUSY;
    if(data == NULL || size == 0) return BMML_INVALID_ARG;

    if(!usart->dma_tx_acquired) {
        int idx = usart_res_idx(usart->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;
        int dma_idx = dma_res_idx(usart_res[idx].dma);
        if(dma_idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(usart_res[idx].dma,
            usart_res[idx].dma_tx_stream, usart_dma_tx_isr, usart, &dma_stream);
        if(st != BMML_OK) return st;

        usart->dma_tx_acquired = true;
        usart->dma_tx.res = &dma_res[dma_idx];
        usart->dma_tx.stream = dma_stream;
        usart->dma_tx.num_stream = usart_res[idx].dma_tx_stream;
        usart->dma_tx.channel = usart_res[idx].dma_tx_channel;

        *usart->dma_tx.res->rcc_reg |= usart->dma_tx.res->rcc_mask;
        dma_disable_stream(usart->dma_tx.stream);
        dma_clear_stream_flags(usart->dma_tx.res->reg, usart->dma_tx.num_stream);
    }

    usart->busy = true;
    usart->tx_phase_done = false;

    usart->dma_tx.stream->PAR  = (uint32_t)&usart->res->reg->DR;
    usart->dma_tx.stream->M0AR = (uint32_t)data;
    usart->dma_tx.stream->NDTR = size;

    uint32_t tx_cr = 0U;
    tx_cr |= (usart->dma_tx.channel << DMA_SxCR_CHSEL_Pos);
    tx_cr |= DMA_SxCR_MINC;
    tx_cr |= DMA_SxCR_DIR_0;  // Memory to peripheral
    tx_cr |= DMA_SxCR_TCIE | DMA_SxCR_TEIE;

    usart->dma_tx.stream->CR = tx_cr;
    usart->dma_tx.stream->CR |= DMA_SxCR_EN;

    usart->res->reg->CR3 |= USART_CR3_DMAT;
    
    return BMML_OK;
}

bmml_status_t usart_dma_start_rx(usart_t *usart, uint8_t *buf, uint16_t buf_len) {
    if(usart == NULL) return BMML_INVALID_ARG;
    if(buf_len <= 0) return BMML_INVALID_ARG;

    usart->dma_rx_tail = 0U;

    if(!usart->dma_rx_acquired) {
        int idx = usart_res_idx(usart->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;
        int dma_idx = dma_res_idx(usart_res[idx].dma);
        if(dma_idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(usart_res[idx].dma,
            usart_res[idx].dma_rx_stream, usart_dma_rx_isr, usart, &dma_stream);
        if(st != BMML_OK) return st;

        usart->dma_rx_acquired = true;
        usart->dma_rx.res = &dma_res[dma_idx];
        usart->dma_rx.stream = dma_stream;
        usart->dma_rx.num_stream = usart_res[idx].dma_rx_stream;
        usart->dma_rx.channel = usart_res[idx].dma_rx_channel;

        *usart->dma_rx.res->rcc_reg |= usart->dma_rx.res->rcc_mask;
        dma_disable_stream(usart->dma_rx.stream);
        dma_clear_stream_flags(usart->dma_tx.res->reg, usart->dma_tx.num_stream);
    }

    usart->dma_rx.stream->PAR  = (uint32_t)&usart->res->reg->DR;
    usart->dma_rx.stream->M0AR = (uint32_t)buf;
    usart->dma_rx.stream->NDTR = buf_len;

    // configure DMA CR
    uint32_t cr = 0U;
    cr |= ((uint32_t)usart->res->dma_rx_channel << DMA_SxCR_CHSEL_Pos);     // select channel
    cr |= DMA_SxCR_MINC;                                                    // enable memory increment
    cr &= ~DMA_SxCR_PINC;                                                   // disable peripheral increment
    cr &= ~DMA_SxCR_DIR;                                                    // direction peripheral to memory
    cr |= DMA_SxCR_CIRC;                                                    // enable circular mode
    cr |= DMA_SxCR_TEIE;

    usart->res->reg->CR1 |= USART_CR1_TE | 
                            USART_CR1_RE | 
                            USART_CR1_IDLEIE | 
                            USART_CR1_UE;

    usart->dma_rx.stream->CR = cr;
    usart->dma_rx.stream->CR |= DMA_SxCR_EN;

    usart->res->reg->CR1 &= ~USART_CR1_RXNEIE;
    // enable DMA in USART and irq IDLE line detection
    usart->res->reg->CR3 |= USART_CR3_DMAR;
    usart->res->reg->CR1 |= USART_CR1_IDLEIE;


    return BMML_OK;
}

static bmml_status_t usart_it_enqueue(usart_t *usart, const uint8_t *data, size_t len) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for(size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        ring_buffer_push(usart->tx_buffer, &b);
    }
    usart->busy = true;
    usart->res->reg->CR1 |= USART_CR1_TXEIE;
    __set_PRIMASK(primask);
    return BMML_OK;
}

bmml_status_t usart_printf(usart_t *usart, const char *format, ...) {
    if(usart == NULL || usart->res == NULL || format == NULL) return BMML_INVALID_ARG;
    if(usart->res->reg->CR3 & USART_CR3_DMAT) return BMML_BUSY;

    char buf[USART_PRINTF_BUF_SIZE];

    va_list args;
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if(n < 0) return BMML_ERROR;
    if(n == 0) return BMML_OK;

    bool truncated = ((size_t)n >= sizeof(buf));
    size_t len = truncated ? sizeof(buf) - 1 : (size_t)n;

    usart_it_enqueue(usart, (const uint8_t *)buf, len);

    return truncated ? BMML_ERROR : BMML_OK;
}

static void USARTx_IRQHandler(int slot) {
    usart_t *usart = &usart_pool[slot];
    uint32_t sr = usart->res->reg->SR;
    uint32_t cr1 = usart->res->reg->CR1;

    // Error flags
    if(sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint32_t err = sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);
        (void)usart->res->reg->SR;       // read SR
        (void)usart->res->reg->DR;       // read DR (reset error flags)
        usart->hw_error = err;
        if(usart->callback != NULL) usart->callback(usart);
    }

    // DMA
    // IDLE line detection
    if((sr & USART_SR_IDLE) && (cr1 & USART_CR1_IDLEIE)) {
        // reset IDLE flag
        (void)usart->res->reg->SR;
        (void)usart->res->reg->DR;

        // calculate the number of bytes received
        uint16_t dma_head = RING_BUF_SIZE - usart->dma_rx.stream->NDTR; // get current head
        uint16_t num_byte = 0U;
        if(dma_head != usart->dma_rx_tail) { // if not empty
            if(dma_head > usart->dma_rx_tail) {
                num_byte = dma_head - usart->dma_rx_tail;
            } else {
                num_byte = (RING_BUF_SIZE - usart->dma_rx_tail) + dma_head;
            }
            // push into ring buffer
            for(uint16_t i = 0; i < num_byte; i++) {
                uint8_t byte = rx_storage[slot][usart->dma_rx_tail];
                ring_buffer_push(usart->rx_buffer, &byte);
                usart->dma_rx_tail++;
                if(usart->dma_rx_tail >= RING_BUF_SIZE) {
                    usart->dma_rx_tail = 0;
                }
            }
        }
        
        if(usart->callback != NULL) {
            usart->callback(usart);
        }
        usart->dma_rx_tail = dma_head;
    }

    // IT
    if((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
        uint8_t ch = usart_receive_byte(usart->res->reg);
        // TODO: add overflow lost data printf
        ring_buffer_push(usart->rx_buffer, &ch);
    } 
    else if((sr & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE)) {
        // send byte
        uint8_t out_data;
        if(ring_buffer_pop(usart->tx_buffer, &out_data)) {
            usart->res->reg->DR = out_data;
        } else {
            usart->res->reg->CR1 &= ~USART_CR1_TXEIE; // buffer is empty, reset TXEIE flag
            usart->busy = false;
            if(usart->callback != NULL) usart->callback(usart);
        }
    }
}

static void usart_abort_error(usart_t *usart, uint32_t hw_error) {
    usart->res->reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
    usart->res->reg->CR1 &= ~USART_CR1_IDLEIE;
    usart->status = BMML_ERROR;
    usart->hw_error = hw_error;
    
    usart->busy = false;

    if(usart->dma_tx_acquired) {
        dma_disable_stream(usart->dma_tx.stream);
        bmml_dma_release_stream(usart->dma_tx.res->reg, usart->dma_tx.num_stream);
        dma_clear_stream_flags(usart->dma_tx.res->reg, usart->dma_tx.num_stream);
        usart->dma_tx_acquired = false;   // см. пункт C
    }
    if(usart->dma_rx_acquired) {
        dma_disable_stream(usart->dma_rx.stream);
        bmml_dma_release_stream(usart->dma_rx.res->reg, usart->dma_rx.num_stream);
        dma_clear_stream_flags(usart->dma_rx.res->reg, usart->dma_rx.num_stream);
        usart->dma_rx_acquired = false;
    }

    if(usart->callback != NULL) usart->callback(usart);
}

static void DMA_finish(usart_t *usart) {
    usart->res->reg->CR3 &= ~USART_CR3_DMAT;
    usart->status = BMML_OK;
    usart->busy   = false;
    if(usart->callback != NULL) usart->callback(usart);
}


static void usart_dma_tx_isr(void *ctx) {
    usart_t *usart = (usart_t *)ctx;
    uint32_t teif = dma_teif_mask(usart->res->tx_tcif_mask);
    if(dma_check_flag(&usart->dma_tx, teif, false)) {
        usart_abort_error(usart, teif);
    } else if(dma_check_flag(&usart->dma_tx, usart->res->tx_tcif_mask, true)) {
        DMA_finish(usart);
    }
}

static void usart_dma_rx_isr(void *ctx) {
    usart_t *usart = (usart_t *)ctx;
    uint32_t teif = dma_teif_mask(usart->res->rx_tcif_mask);
    if(dma_check_flag(&usart->dma_rx, teif, false)) {
        usart_abort_error(usart, teif);
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