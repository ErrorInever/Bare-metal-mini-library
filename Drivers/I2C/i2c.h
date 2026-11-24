#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f446xx.h"
#include "bmml_res_i2c.h"
#include "bmml_status.h"
#include "bmml_dma_shared.h"

typedef struct i2c_t i2c_t;

/** @brief FSM position, updated from the event/error/DMA ISRs. */
typedef enum {
    I2C_IDLE,
    I2C_START,   /**< START/Sr requested, waiting for SB.                     */
    I2C_ADDR,    /**< Address sent, waiting for ADDR (slave ACK).             */
    I2C_TX,      /**< Sending data (IT: TXE/BTF, DMA: waiting for TC + BTF).  */
    I2C_RX,      /**< Receiving data (IT: RXNE/BTF, DMA: waiting for TC).     */
    I2C_STOP,    /**< Reserved.                                               */
    I2C_DONE,    /**< Last transaction finished successfully.                 */
    I2C_ERROR    /**< Last transaction failed, see status / hw_error.         */
} i2c_state_t;

typedef enum {
    I2C_SM_100KHZ,
    I2C_FM_400KHZ   /**< Fm, DUTY = 0 (Tlow/Thigh = 2). Needs PCLK1 >= 4 MHz. */
} i2c_mode_t;

/**
 * @brief One bus transaction.
 *
 * Supported combinations:
 *  - tx_len > 0, rx_len == 0 : write
 *  - tx_len == 0, rx_len > 0 : read
 *  - tx_len > 0, rx_len > 0  : write, repeated START, read. repeated_start MUST be true.
 *
 * Buffers must stay valid until the callback is called. Length limit is 65535 for DMA.
 * On NACK the WHOLE transaction is restarted from the beginning, up to max_retries times.
 */
typedef struct {
    uint16_t       addr;            /**< Raw 7-bit address (NOT pre-shifted). */
    const uint8_t *tx_buff;
    size_t         tx_len;
    uint8_t       *rx_buff;
    size_t         rx_len;
    bool           repeated_start;
    uint8_t        max_retries;
} i2c_transaction_t;

typedef enum { I2C_XFER_IT, I2C_XFER_DMA } i2c_xfer_mode_t;

/** @brief Called from ISR context once per transaction. Check i2c->status / i2c->hw_error. */
typedef void (*i2c_callback_t)(i2c_t *i2c);

struct i2c_t {
    const i2c_res_t *res;
    dma_t dma_tx;
    dma_t dma_rx;
    bool dma_tx_acquired;
    bool dma_rx_acquired;

    volatile i2c_state_t   state;
    volatile bmml_status_t status;     /**< BMML_OK or BMML_ERROR / BMML_TIMEOUT after the callback. */
    volatile uint32_t      hw_error;   /**< I2C_SR1_xx bit(s) or DMA TEIF mask when status != OK.    */

    i2c_xfer_mode_t   xfer_mode;
    i2c_transaction_t ctx;             /**< Private copy of the active transaction. */
    uint8_t  retry_cnt;
    size_t   tx_cnt;
    size_t   rx_cnt;
    bool     rx_phase;                 /**< true: current phase is a read. */
    volatile bool busy;
    i2c_callback_t callback;
};

bmml_status_t i2c_acquire(I2C_TypeDef *reg, i2c_mode_t mode, i2c_callback_t cb, i2c_t **out);
bmml_status_t i2c_release(i2c_t *i2c);

/** @brief Interrupt-driven transfer (write, read or write+read). */
bmml_status_t i2c_it_transmit(i2c_t *i2c, i2c_transaction_t *tr);
/** @brief DMA transfer (write or write+read). Also accepts read-only. */
bmml_status_t i2c_dma_transmit(i2c_t *i2c, i2c_transaction_t *tr);
/** @brief DMA read-only transfer: requires tx_len == 0 and rx_len > 0. */
bmml_status_t i2c_dma_receive(i2c_t *i2c, i2c_transaction_t *tr);

static inline void i2c_reset_off(I2C_TypeDef *reg) { reg->CR1 &= ~I2C_CR1_PE; }
static inline void i2c_enable(I2C_TypeDef *reg)    { reg->CR1 |= I2C_CR1_PE; }

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */