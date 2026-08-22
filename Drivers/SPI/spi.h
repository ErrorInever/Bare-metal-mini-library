#ifndef BMML_SPI_H
#define BMML_SPI_H


#include "stm32f446xx.h"
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
    volatile bmml_status_t status;
    volatile uint32_t hw_error;

    /* IRQ Context tracking variables */
    volatile uint8_t *p_tx_ptr;                /**< Internal tracking pointer for shifting Tx bytes. */
    volatile uint8_t *p_rx_ptr;                /**< Internal tracking pointer for storing Rx bytes. */
    volatile uint16_t tx_cnt;                  /**< Amount of remaining bytes to load into DR. */
    volatile uint16_t rx_cnt;                  /**< Amount of remaining bytes to read from DR. */

    volatile spi_transaction_t ctx;
    volatile spi_transaction_t *current_ctx;
    bool busy;
    spi_callback_t callback;

} spi_t;

bmml_status_t spi_acquire(SPI_TypeDef *reg, spi_baudrate_t br, spi_callback_t cb);
bmml_status_t spi_release();
bmml_status_t spi_it_transmit(spi_t *spi, spi_transaction_t *tr);
bmml_status_t spi_dma_transmit(spi_t *spi, spi_transaction_t *tr);

static inline void spi_reset(SPI_TypeDef *reg) {
    reg->CR1 = 0U;
}

static inline void spi_enable(SPI_TypeDef *reg) {
    reg->CR1 |= SPI_CR1_SPE;
}

static inline void spi_reset_flags(SPI_TypeDef *reg) {
    reg->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
}


#ifdef __cplusplus
}
#endif

#endif /*BMML_SPI_H*/