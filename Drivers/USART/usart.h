#ifndef USART_H
#define USART_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f446xx.h"
#include "bmml_res_usart.h"
#include "bmml_dma_shared.h"
#include "bmml_status.h"
#include "bmml_ring_buffer.h"

struct usart_t;
typedef struct usart_t usart_t;

typedef enum {
    USART_EVENT_RX,        /**< New data is available in rx_buffer (read it with usart_read). */
    USART_EVENT_TX_DONE,   /**< The last byte has left the shift register. */
    USART_EVENT_ERROR      /**< Line error (ORE/NE/FE/PE) or DMA error, see usart->hw_error. */
} usart_event_t;

typedef void (*usart_callback_t)(usart_t *usart, usart_event_t event);

typedef struct usart_t {
    const usart_res_t *res;
    dma_t dma_tx;
    dma_t dma_rx;
    bool dma_tx_acquired;
    bool dma_rx_acquired;
    volatile bmml_status_t status;
    volatile uint32_t hw_error;

    RingBuffer_t *tx_buffer;       /**< IT transmit queue (ISR is the consumer). */
    RingBuffer_t *rx_buffer;       /**< Received bytes, filled by the IT or DMA path. */

    uint8_t *dma_rx_buf;           /**< User buffer the RX DMA writes into (circular). */
    uint16_t dma_rx_len;
    uint16_t dma_rx_tail;          /**< Index of the next byte not yet moved to rx_buffer. */
    volatile bool dma_rx_active;

    volatile bool busy;            /**< A transmission is in progress (IT or DMA). */
    usart_callback_t callback;
} usart_t;

bmml_status_t usart_acquire(USART_TypeDef *reg, uint32_t br, usart_callback_t cb, usart_t **out);
bmml_status_t usart_release(usart_t *usart);

/** Queue a zero-terminated string. All-or-nothing: BMML_BUSY if it does not fit or a DMA TX is running. */
bmml_status_t usart_it_transmit(usart_t *usart, const char *data);
/** DMA transmit. `data` must stay valid until USART_EVENT_TX_DONE. */
bmml_status_t usart_dma_transmit(usart_t *usart, const char *data, uint16_t size);

/** Start circular DMA reception. `buf` must be static/global. Data is moved to rx_buffer on
 *  half/full buffer and on IDLE line. */
bmml_status_t usart_dma_start_rx(usart_t *usart, uint8_t *buf, uint16_t buf_len);
bmml_status_t usart_dma_stop_rx(usart_t *usart);

/** Copy up to `max` received bytes out of rx_buffer. Returns the number of bytes copied. */
size_t usart_read(usart_t *usart, uint8_t *dst, size_t max);

static inline size_t usart_available(const usart_t *usart) {
    return ring_buffer_count(usart->rx_buffer);
}

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