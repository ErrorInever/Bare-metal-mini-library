#ifndef BMML_SPI_H
#define BMML_SPI_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bmml_dma_shared.h"
#include "bmml_res_spi.h"
#include "bmml_status.h"


/* Forward declarations */
struct spi_t;
struct spi_transaction_t;

typedef struct spi_t spi_t;
typedef struct spi_transaction_t spi_transaction_t;


/**
 * @enum spi_baudrate_t
 * @brief SPI Clock Prescaler options mapping directly to the CR1->BR configuration bits.
 * @note F_sck = F_pclk / Divider.
 */
typedef enum {
    SPI_BAUDRATE_DIV2   = 0, /**< PCLK divided by 2 */
    SPI_BAUDRATE_DIV4   = 1, /**< PCLK divided by 4 */
    SPI_BAUDRATE_DIV8   = 2, /**< PCLK divided by 8 */
    SPI_BAUDRATE_DIV16  = 3, /**< PCLK divided by 16 */
    SPI_BAUDRATE_DIV32  = 4, /**< PCLK divided by 32 */
    SPI_BAUDRATE_DIV64  = 5, /**< PCLK divided by 64 */
    SPI_BAUDRATE_DIV128 = 6, /**< PCLK divided by 128 */
    SPI_BAUDRATE_DIV256 = 7  /**< PCLK divided by 256 */
} spi_baudrate_t;

/**
 * @enum spi_state_t
 * @brief Internal state machine statuses for tracking the ongoing bus operations.
 */
typedef enum {
    SPI_READY,          /**< Bus is idle and ready for a new transaction. */
    SPI_BUSY_TX,        /**< Transmit-only operation is in progress. */
    SPI_BUSY_RX,        /**< Receive-only operation is in progress (sending dummy bytes). */
    SPI_BUSY_TX_RX,     /**< Full-duplex bidirectional exchange is in progress. */
    SPI_ERROR           /**< Peripheral error occurred, blocking operations. */
} spi_state_t;

/**
 * @brief Asynchronous completion callback type definition.
 * * @param spi Pointer to the SPI handle managing the operation.
 */
typedef void (*spi_callback_t)(spi_t *spi);

/**
 * @struct spi_transaction_t
 * @brief Defines a data package configuration to be transmitted or received over the SPI bus.
 */
typedef struct spi_transaction_t {
    const uint8_t *tx_buff;  /**< Pointer to transmit buffer. Set to NULL for Rx-only mode. */
    uint16_t tx_len;        /**< Size of transmit data in bytes. */
    uint8_t *rx_buff;        /**< Pointer to receive buffer. Set to NULL for Tx-only mode. */
    uint16_t rx_len;        /**< Size of expected receive data in bytes. */
    spi_callback_t callback;/**< Optional completion callback invoked inside the ISR context. */
} spi_transaction_t;

typedef struct spi_t {
    const spi_res_t *res;
    dma_t dma_tx;                        /**< DMA stream for TX */
    dma_t dma_rx;                        /**< DMA stream for RX */
    bool dma_tx_acquired;
    bool dma_rx_acquired;
    volatile spi_state_t state;
    volatile bmml_status_t status;
    volatile uint32_t hw_error;

    /* IRQ Context tracking variables */
    volatile uint8_t *p_tx_ptr;                /**< Internal tracking pointer for shifting Tx bytes. */
    volatile uint8_t *p_rx_ptr;                /**< Internal tracking pointer for storing Rx bytes. */
    volatile uint16_t tx_cnt;                  /**< Amount of remaining bytes to load into DR. */
    volatile uint16_t rx_cnt;                  /**< Amount of remaining bytes to read from DR. */

    volatile spi_transaction_t ctx;
    volatile spi_transaction_t *current_ctx;

} spi_t;

bmml_status_t spi_acquire();
bmml_status_t spi_release();
bmml_status_t spi_it_transmit();
bmml_status_t spi_dma_transmit();


#ifdef __cplusplus
}
#endif

#endif /*BMML_SPI_H*/