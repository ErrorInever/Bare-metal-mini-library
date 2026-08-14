#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif


#include "bmml_res_i2c.h"
#include "bmml_status.h"
#include "bmml_dma_shared.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>


struct i2c_t;
typedef struct i2c_t i2c_t;

/** * @enum i2c_state_t
 * @brief Finite State Machine (FSM) positions for non-blocking interrupt tracking.
 */
typedef enum {
    I2C_IDLE,             /**< Peripheral is relaxed; no active communication on the bus. */
    I2C_START,            /**< Start condition generated; waiting for SB (Start Bit) hardware flag. */
    I2C_ADDR,             /**< Slave Address byte transmitted; waiting for ADDR match clearance flag. */
    I2C_TX,               /**< Asynchronous data transmission loop actively flushing data from RAM. */
    I2C_RX,               /**< Asynchronous data reception loop actively capturing incoming bytes. */
    I2C_STOP,             /**< Stop condition commanded; waiting for bus release verification. */
    I2C_DONE,             /**< FSM successfully reached its valid terminal condition. */
    I2C_ERROR             /**< Exception encountered; running hardware recovery routines. */
} i2c_state_t;

/** * @enum i2c_mode_t
 * @brief SCL clock frequency operational standards.
 */
typedef enum {
    I2C_SM_100KHZ,        /**< Standard Mode communication speed up to 100 kHz. */
    I2C_FM_400KHZ         /**< Fast Mode communication speed up to 400 kHz. */
} i2c_mode_t;

/** * @struct i2c_transaction_t
 * @brief Communication Context Profile containing memory addresses and control switches.
 */
typedef struct {
    uint16_t addr;          /**< 7-bit Target Slave Address. @note Pass a raw 7-bit value; do not pre-shift left. */
    uint8_t *tx_buff;       /**< Pointer to the source data buffer in RAM for transmission. */
    size_t tx_len;          /**< Total number of data bytes allocated for transmission. */
    uint8_t *rx_buff;       /**< Pointer to the destination buffer in RAM for storing received bytes. */
    size_t rx_len;          /**< Total number of data bytes expected to be read. */
    bool repeated_start;    /**< True: Issue Repeated Start instead of a Stop condition between TX and RX phases. */
    uint8_t max_retries;    /**< Max number of automatic transmission restarts if the hardware receives a NACK. */
} i2c_transaction_t;

typedef enum { I2C_XFER_IT, I2C_XFER_DMA } i2c_xfer_mode_t;

typedef void (*i2c_callback_t)(i2c_t *i2c);

typedef struct i2c_t {
    const i2c_res_t *res;
    dma_t dma_tx;                        /**< DMA stream for TX */
    dma_t dma_rx;                        /**< DMA stream for RX */
    bool dma_tx_acquired;
    bool dma_rx_acquired;
    volatile i2c_state_t state;          /**< Volatile FSM tracking state evaluated within ISRs. */
    volatile bmml_status_t status;
    i2c_xfer_mode_t xfer_mode;          // which path in current transaction
    volatile uint32_t hw_error;
    i2c_transaction_t ctx;               /**< Internal clone layout of the active operational context. */
    i2c_transaction_t *current_ctx;      /**< Persistent master handle reference pointer used for retry setups. */
    uint8_t retry_cnt;                   /**< Current iteration tally of triggered recovery retries. */
    uint16_t tx_cnt;                     /**< current tx index  */
    uint16_t rx_cnt;                     /**< current rx index */
    volatile bool busy;                  /**< busy flag */
    i2c_callback_t callback;             /**< Registered user event callback triggered on completion or failure. */
} i2c_t;



bmml_status_t i2c_acquire(I2C_TypeDef *reg, i2c_mode_t mode, i2c_callback_t cb, i2c_t **out);
bmml_status_t i2c_release(i2c_t *i2c);
bmml_status_t i2c_it_transmit(i2c_t *i2c, i2c_transaction_t *tr);

bmml_status_t i2c_dma_transmit(i2c_t *i2c, i2c_transaction_t *tr);
bmml_status_t i2c_dma_receive(i2c_t *i2c, i2c_transaction_t *tr);

static inline void i2c_reset_off(I2C_TypeDef *reg) {
    reg->CR1 &= ~I2C_CR1_PE;
}

static inline void i2c_enable(I2C_TypeDef *reg) {
    reg->CR1 |= I2C_CR1_PE;
}




#ifdef __cplusplus
}
#endif


#endif /* I2C_H */
