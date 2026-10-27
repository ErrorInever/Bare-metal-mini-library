#ifndef USART_H
#define USART_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"
#include "bmml_res_usart.h"
#include "bmml_dma_shared.h"
#include "bmml_status.h"
#include "bmml_ring_buffer.h"

struct usart_t;
typedef struct usart_t usart_t;

typedef void (*usart_callback_t)(usart_t *usart);

typedef struct usart_t {
    const usart_res_t *res;
    dma_t dma_tx;
    dma_t dma_rx;
    bool dma_tx_acquired;
    bool dma_rx_acquired;
    volatile bmml_status_t status;
    volatile uint32_t hw_error;

    RingBuffer_t *tx_buffer;
    RingBuffer_t *rx_buffer;
    uint8_t dma_rx_tail;

    volatile bool tx_phase_done;
    volatile bool rx_phase_done;

    bool busy;
    usart_callback_t callback;
} usart_t;

bmml_status_t usart_acquire(USART_TypeDef *reg, uint32_t br, usart_callback_t cb, usart_t **out);
bmml_status_t usart_release(usart_t *usart);
bmml_status_t usart_it_transmit(usart_t *usart, const char *data);
bmml_status_t usart_dma_transmit(usart_t *usart, const char *data, uint16_t size);
bmml_status_t usart_dma_start_rx(usart_t *usart, uint8_t *buf, uint16_t buf_len);

bmml_status_t usart_printf(usart_t *usart, const char *format, ...)__attribute__((format(printf, 2, 3)));

/**
 * @brief Inline utility mapping an immediate 8-bit read directly off the hardware data register bus structure.
 * * @param[in] instance Direct pointer targeting a valid hardware USART memory map interface register group.
 * @return Extracted raw character code byte from the lower bits of the Data Register (DR).
 */
static inline uint8_t usart_receive_byte(const USART_TypeDef *reg) {
    return reg->DR & 0xFF;
}

static inline void usart_enable(USART_TypeDef *reg) {
    reg->CR1 |= USART_CR1_UE;
}

static inline void usart_disable(USART_TypeDef *reg) {
    reg->CR1 &= ~USART_CR1_UE;
}

#ifdef __cplusplus
}
#endif

#endif /*USART_H*/