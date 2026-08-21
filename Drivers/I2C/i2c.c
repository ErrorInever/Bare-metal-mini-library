#include "i2c.h"
#include "bmml_dma_shared.h"
#include "bmml_res_i2c.h"
#include "bmml_status.h"
#include "bmml_utilities.h"
#include "stm32f446xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

#define I2C_COUNT 3

static i2c_t i2c_pool[I2C_COUNT];
static bool i2c_taken[I2C_COUNT];

static int i2c_slot(const I2C_TypeDef *reg) {
    if(reg == I2C1) return 0;
    if(reg == I2C2) return 1;
    if(reg == I2C3) return 2;
    return -1;
}

bmml_status_t i2c_acquire(I2C_TypeDef *reg, i2c_mode_t mode, i2c_callback_t cb, i2c_t **out) {
    if(out) *out = NULL;
    if(reg == NULL) return BMML_INVALID_ARG;
    if(mode != I2C_SM_100KHZ && mode != I2C_FM_400KHZ) return BMML_INVALID_ARG;

    int slot = i2c_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(i2c_taken[slot]) return BMML_BUSY;

    int idx = i2c_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    // Init I2C object
    i2c_taken[slot] = true;
    i2c_t *i2c = &i2c_pool[slot];
    i2c->res = &i2c_res[idx];
    i2c->callback = cb;

    // RCC
    *i2c->res->rcc_reg |= i2c->res->rcc_mask;

    i2c_reset_off(reg);

    // Configure freq
    uint32_t freq_mhz = get_apb1_clock_hz() / 1000000U;
    if (freq_mhz < 2U || freq_mhz > 50U) return BMML_INVALID_ARG;
    i2c->res->reg->CR2 &= ~I2C_CR2_FREQ;
    i2c->res->reg->CR2 |= (freq_mhz & I2C_CR2_FREQ);

    // reset CCR & TRISE
    i2c->res->reg->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR);
    i2c->res->reg->TRISE &= ~I2C_TRISE_TRISE;

    if(mode == I2C_SM_100KHZ) {
        // Standard Mode (100 kHz)
        // CCR = F_pclk1 / (2 * 100 000) -> multiple 1 000 000 and then devide by 1000
        uint32_t ccr_val = (freq_mhz * 1000U) / 200U;
        if (ccr_val < 4) ccr_val = 4; // The minimum allowed value is 0x04, except when Duty = 1 (c)
        i2c->res->reg->CCR |= (ccr_val & I2C_CCR_CCR);
        // Calculate TRISE
        // TRISE = (1000ns / pclk1_mhz) + 1
        i2c->res->reg->TRISE |= ((freq_mhz + 1U) & I2C_TRISE_TRISE);
    } else {
        // Fast Mode (400 kHz)
        // Enable Fast mode bit and setup DUTY = 0. t_low/t_high = 2
        i2c->res->reg->CCR |= I2C_CCR_FS;
        // For DUTY = 0: CCR = F_pclk1 / (3 * 400 000)
        uint32_t ccr_val = (freq_mhz * 1000U) / 1200U;
        if (ccr_val < 1) ccr_val = 1;
        i2c->res->reg->CCR |= (ccr_val & I2C_CCR_CCR);
        // TRISE: For Fast Mode max time rise 300ns
        // (300ns * F_pclk1) + 1 = (0.3 * pclk1_mhz) + 1
        uint32_t trise_val = ((freq_mhz * 300U) / 1000U) + 1U;
        i2c->res->reg->TRISE |= (trise_val & I2C_TRISE_TRISE);
    }

    // Enable interrupts
    NVIC_EnableIRQ(i2c->res->irqn_ev);
    NVIC_SetPriority(i2c->res->irqn_ev, 5);
    NVIC_EnableIRQ(i2c->res->irqn_er);
    NVIC_SetPriority(i2c->res->irqn_er, 5);
    // Enable streams
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);


    i2c_enable(reg);

    return BMML_OK;
}

bmml_status_t i2c_release(i2c_t *i2c) {
    if(i2c == NULL || i2c->res == NULL) return BMML_INVALID_ARG;

    if(i2c->busy) return BMML_BUSY;

    int slot = i2c_slot(i2c->res->reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(!i2c_taken[slot]) return BMML_INVALID_ARG;

    i2c_reset_off(i2c->res->reg);
    i2c->res->reg->CR2 &= ~I2C_CR2_FREQ;
    i2c->res->reg->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR);

    i2c_taken[slot] = false;
    i2c_pool[slot] = (i2c_t){0};

    NVIC_DisableIRQ(i2c->res->irqn_ev);
    NVIC_DisableIRQ(i2c->res->irqn_er);

    NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    NVIC_DisableIRQ(DMA1_Stream7_IRQn);

    if (i2c->dma_tx_acquired) {
        if((dma_disable_stream(i2c->dma_tx.stream)) != BMML_OK) return BMML_TIMEOUT;
        bmml_dma_release_stream(i2c->dma_tx.res->reg, i2c->dma_tx.num_stream);
        dma_clear_stream_flags(i2c->dma_tx.res->reg, i2c->dma_tx.num_stream);
    }
    if (i2c->dma_rx_acquired) {
        if((dma_disable_stream(i2c->dma_rx.stream)) != BMML_OK) return BMML_TIMEOUT;
        bmml_dma_release_stream(i2c->dma_rx.res->reg, i2c->dma_rx.num_stream);
        dma_clear_stream_flags(i2c->dma_rx.res->reg, i2c->dma_rx.num_stream);
    }

    return BMML_OK;
}

bmml_status_t i2c_it_transmit(i2c_t *i2c, i2c_transaction_t *tr) {
    if(i2c == NULL || i2c->res == NULL) return BMML_INVALID_ARG;
    if(i2c->busy) return BMML_BUSY;
    if (tr == NULL) return BMML_INVALID_ARG;
    if (tr->tx_len == 0 && tr->rx_len == 0) return BMML_INVALID_ARG;
    if (tr->tx_len > 0 && tr->tx_buff == NULL) return BMML_INVALID_ARG;
    if (tr->rx_len > 0 && tr->rx_buff == NULL) return BMML_INVALID_ARG;

    // IT transaction mode
    i2c->xfer_mode = I2C_XFER_IT;

    i2c->busy = true;
    i2c->ctx = *tr;
    i2c->current_ctx = tr;
    i2c->tx_cnt = 0;
    i2c->rx_cnt = 0;
    i2c->retry_cnt = 0;
    i2c->state = I2C_START;
    i2c->status = BMML_OK;
    i2c->hw_error = 0;

    // Enable event and error interrupt
    i2c->res->reg->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    i2c->res->reg->CR1 |= I2C_CR1_START; // Start Generation

    return BMML_OK;
}

bmml_status_t i2c_dma_transmit(i2c_t *i2c, i2c_transaction_t *tr) {
    if (i2c == NULL || i2c->res == NULL) return BMML_INVALID_ARG;
    if (i2c->busy) return BMML_BUSY;
    if (tr == NULL) return BMML_INVALID_ARG;
    if (tr->tx_len == 0 && tr->rx_len == 0) return BMML_INVALID_ARG;
    if (tr->tx_len > 0 && tr->tx_buff == NULL) return BMML_INVALID_ARG;
    if (tr->rx_len > 0 && tr->rx_buff == NULL) return BMML_INVALID_ARG; 

    // DMA TX acquired
    if(tr->tx_len > 0 && !i2c->dma_tx_acquired) {
        int dma_idx = dma_res_idx(DMA1);
        if(dma_idx < 0) return BMML_INVALID_ARG;
        int idx = i2c_res_idx(i2c->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(DMA1, i2c_res[idx].dma_tx_stream, &dma_stream);
        if(st != BMML_OK) return st;

        i2c->dma_tx_acquired = true;
        i2c->dma_tx.res = &dma_res[dma_idx];
        i2c->dma_tx.stream = dma_stream;
        i2c->dma_tx.num_stream = i2c_res[idx].dma_tx_stream;
        i2c->dma_tx.channel = i2c_res[idx].dma_tx_channel;

        *i2c->dma_tx.res->rcc_reg |= i2c->dma_tx.res->rcc_mask;
        dma_disable_stream(i2c->dma_tx.stream);
    } 

    // DMA RX acquired if needed
    if(tr->rx_len > 0 && !i2c->dma_rx_acquired) {
        int dma_idx = dma_res_idx(DMA1);
        if(dma_idx < 0) return BMML_INVALID_ARG;
        int idx = i2c_res_idx(i2c->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(DMA1, i2c_res[idx].dma_rx_stream, &dma_stream);
        if(st != BMML_OK) return st;

        i2c->dma_rx_acquired = true;
        i2c->dma_rx.res = &dma_res[dma_idx];
        i2c->dma_rx.stream = dma_stream;
        i2c->dma_rx.num_stream = i2c_res[idx].dma_rx_stream;
        i2c->dma_rx.channel = i2c_res[idx].dma_rx_channel;

        *i2c->dma_rx.res->rcc_reg |= i2c->dma_rx.res->rcc_mask;
        dma_disable_stream(i2c->dma_rx.stream);
    }

    i2c->xfer_mode = I2C_XFER_DMA;
    i2c->busy = true;
    i2c->ctx = *tr;
    i2c->current_ctx = tr;
    i2c->tx_cnt = 0;
    i2c->rx_cnt = 0;
    i2c->retry_cnt = 0;
    i2c->status = BMML_OK;
    i2c->hw_error = 0;

    // Setup addresses
    // (Data Holding Register, 12-bit, Right-aligned, Channel 1

    // DMA TX config
    uint32_t cr = 0U;
    cr |= (i2c->dma_tx.channel << DMA_SxCR_CHSEL_Pos);          // select channel (7)
    cr |= DMA_SxCR_MINC;                                        // increment memory
    cr &= ~DMA_SxCR_PINC;                                       // disable peref increment
    cr |= (1U << DMA_SxCR_DIR_Pos);                             // direction M2P
    cr |= DMA_SxCR_TCIE;                                        // enable interrupt Transmit complete
    cr |= DMA_SxCR_TEIE;                                        // enable interrupt Transport error
    i2c->dma_tx.stream->CR = cr;
    // DMA RX config
    cr = 0U;
    cr |= (i2c->dma_rx.channel << DMA_SxCR_CHSEL_Pos);          // select channel (7)
    cr |= DMA_SxCR_MINC;                                        // increment memory
    cr &= ~DMA_SxCR_PINC;                                       // disable peref increment
    cr |= 0U;                                                   // direction P2M
    cr |= DMA_SxCR_TCIE;                                        // enable interrupt Transmit complete
    cr |= DMA_SxCR_TEIE;                                        // enable interrupt Transport error
    i2c->dma_rx.stream->CR = cr;


    i2c->state = I2C_START;
    i2c->res->reg->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    i2c->res->reg->CR1 |= I2C_CR1_START;

    return BMML_OK;
}


bmml_status_t i2c_dma_receive(i2c_t *i2c, i2c_transaction_t *tr) {
    if (i2c == NULL || i2c->res == NULL) return BMML_INVALID_ARG;
    if (i2c->busy) return BMML_BUSY;
    if (tr == NULL) return BMML_INVALID_ARG;
    if (tr->rx_len == 0) return BMML_INVALID_ARG;

    // DMA RX acquired
    if(!i2c->dma_rx_acquired) {
        int dma_idx = dma_res_idx(DMA1);
        if(dma_idx < 0) return BMML_INVALID_ARG;
        int idx = i2c_res_idx(i2c->res->reg);
        if(idx < 0) return BMML_INVALID_ARG;

        DMA_Stream_TypeDef *dma_stream;
        bmml_status_t st = bmml_dma_acquire_stream(DMA1, i2c_res[idx].dma_rx_stream, &dma_stream);
        if(st != BMML_OK) return st;

        i2c->dma_rx_acquired = true;
        i2c->dma_rx.res = &dma_res[dma_idx];
        i2c->dma_rx.stream = dma_stream;
        i2c->dma_rx.num_stream = i2c_res[idx].dma_rx_stream;
        i2c->dma_rx.channel = i2c_res[idx].dma_rx_channel;

        *i2c->dma_rx.res->rcc_reg |= i2c->dma_rx.res->rcc_mask;
        dma_disable_stream(i2c->dma_rx.stream);
    }


    i2c->xfer_mode = I2C_XFER_DMA;
    i2c->busy = true;
    i2c->ctx = *tr;
    i2c->current_ctx = tr;
    i2c->rx_cnt = 0;
    i2c->retry_cnt = 0;
    i2c->status = BMML_OK;
    i2c->hw_error = 0;

    // DMA RX config
    uint32_t cr = 0U;
    cr |= (i2c->dma_rx.channel << DMA_SxCR_CHSEL_Pos);          // select channel (7)
    cr |= DMA_SxCR_MINC;                                        // increment memory
    cr &= ~DMA_SxCR_PINC;                                       // disable peref increment
    cr |= 0U;                                                   // direction P2M
    cr |= DMA_SxCR_TCIE;                                        // enable interrupt Transmit complete
    cr |= DMA_SxCR_TEIE;                                        // enable interrupt Transport error
    i2c->dma_rx.stream->CR = cr;

    i2c->state = I2C_START;
    i2c->res->reg->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    i2c->res->reg->CR1 |= I2C_CR1_START;

    return BMML_OK;
}


void I2Cx_EV_IRQ_execute(int slot) {
    assert(slot >= 0);

    i2c_t *i2c = &i2c_pool[slot];
    uint32_t sr1 = i2c->res->reg->SR1;

    bool transaction_finished = false;
    // I2C FSM
    switch (i2c->state) {
        case I2C_START:
            // Start bit was generated successfull
            if(sr1 & I2C_SR1_SB) {
                i2c->state = I2C_ADDR; // next state
                // reset flag and define read (0x01) or write (0x00) byte
                uint8_t raw_bit = (i2c->ctx.tx_len > 0) ? 0x00U : 0x01U;
                i2c->res->reg->DR = (i2c->ctx.addr << 1) | raw_bit; // FIXME: WARNING: 7 bit address (mb not need addr << 1)
            }
            break;

        case I2C_ADDR:
            // if address send successfull and get ACK from slave
            if(sr1 & I2C_SR1_ADDR) {
                (void)i2c->res->reg->SR2; // reset addr (read sr1 &sr2)
                // if we want to send
                if(i2c->ctx.tx_len > 0) {
                    i2c->state = I2C_TX; // state now TX
                    // If mode is DMA
                    if(i2c->xfer_mode == I2C_XFER_DMA) {
                        // Configure and enable DMA TX-stream (PAR=&DR, M0AR=tx_buff, NDTR=tx_len)
                        i2c->dma_tx.stream->PAR = (uint32_t)&i2c->res->reg->DR;
                        i2c->dma_tx.stream->M0AR = (uint32_t)i2c->ctx.tx_buff;
                        i2c->dma_tx.stream->NDTR = i2c->ctx.tx_len;
                        i2c->res->reg->CR2 |= I2C_CR2_DMAEN;
                        i2c->dma_tx.stream->CR |= DMA_SxCR_EN;
                    } else {  // if mode is Interrupt for TX
                        i2c->res->reg->CR2 |= I2C_CR2_ITBUFEN; // Buffer Interrupt Enable TXE
                        i2c->res->reg->DR = i2c->ctx.tx_buff[i2c->tx_cnt++]; // Send a first byte
                    }
                } else if(i2c->ctx.rx_len > 0) { // if we want to read
                    i2c->state = I2C_RX; // state now RX
                    // If mode is DMA
                    if(i2c->xfer_mode == I2C_XFER_DMA) {
                        // Configure and enable DMA RX-stream (PAR=&DR, M0AR=rx_buff, NDTR=rx_len)
                        i2c->dma_rx.stream->PAR = (uint32_t)&i2c->res->reg->DR;
                        i2c->dma_rx.stream->M0AR = (uint32_t)i2c->ctx.rx_buff;
                        i2c->dma_rx.stream->NDTR = i2c->ctx.rx_len;

                        if(i2c->ctx.rx_len == 1) {
                            i2c->res->reg->CR1 &= ~I2C_CR1_ACK;
                        } else {
                            i2c->res->reg->CR2 |= I2C_CR2_LAST;   // last DMA-byte need NACK
                            i2c->res->reg->CR1 |= I2C_CR1_ACK;
                        }
                        i2c->res->reg->CR2 |= I2C_CR2_DMAEN;
                        i2c->dma_rx.stream->CR |= DMA_SxCR_EN;
                    } else { // if mode is Interrupt for RX
                        i2c->res->reg->CR2 |= I2C_CR2_ITBUFEN; // Buffer Interrupt Enable RXNE
                        // If we need only 1 byte, we must immediately disable ACK, in accordance with the specification.
                        if(i2c->ctx.rx_len == 1) {
                            i2c->res->reg->CR1 &= ~I2C_CR1_ACK;
                            i2c->res->reg->CR1 |= I2C_CR1_STOP;
                        } else {
                            i2c->res->reg->CR1 |= I2C_CR1_ACK;
                        }
                    }
                }
            }
            break;

        case I2C_TX:
            // if end transfer and it's IT mode
            if(i2c->xfer_mode == I2C_XFER_IT) {
                if(sr1 & I2C_SR1_BTF) {
                    if(i2c->tx_cnt >= i2c->ctx.tx_len) {
                        if(i2c->ctx.repeated_start && i2c->ctx.rx_len > 0) {
                            i2c->ctx.tx_len = 0; 
                            i2c->state = I2C_START; 
                            i2c->res->reg->CR1 |= I2C_CR1_START;
                        } else {
                            i2c->res->reg->CR1 |= I2C_CR1_STOP;
                            i2c->res->reg->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
                            i2c->state = I2C_DONE;
                            i2c->status = BMML_OK;
                            i2c->busy = false;
                            transaction_finished = true;
                        }
                        break;
                    }
                }
            }
            // if BTF = 0 and DR is empty
            if(sr1 & I2C_SR1_TXE) {
                if(i2c->tx_cnt < i2c->ctx.tx_len) {
                    i2c->res->reg->DR = i2c->ctx.tx_buff[i2c->tx_cnt++];
                } else { 
                    // in previous step we copy last byte and now it in shift register
                    // just disable ITBUFFEN interrupt and wait BTF = 1
                    i2c->res->reg->CR2 &= ~I2C_CR2_ITBUFEN;
                }
            }
            break;

        case I2C_RX:
            // DR is not empty, read next byte
            if(i2c->xfer_mode == I2C_XFER_IT) {
                if(sr1 & I2C_SR1_RXNE) {
                    // Remaining number of bytes (including DR)
                    size_t remaining = i2c->ctx.rx_len - i2c->rx_cnt;
                    if(remaining == 2) {
                        // If we read the penultimate byte now, the shift register
                        // will start receiving the very last one. We must disable ACK in advance
                        i2c->res->reg->CR1 &= ~I2C_CR1_ACK;
                        // Also, according to the RM specification, if we want to generate a STOP 
                        // condition immediately after the last byte, the STOP command 
                        // can be set right now.
                        i2c->res->reg->CR1 |= I2C_CR1_STOP;    
                    }
                    i2c->ctx.rx_buff[i2c->rx_cnt++] = i2c->res->reg->DR; // RXNE autoreset after read DR
                    // if it was the last byte stop transaction
                    if(i2c->rx_cnt >= i2c->ctx.rx_len) {
                        i2c->res->reg->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
                        i2c->state = I2C_DONE;
                        i2c->status = BMML_OK;
                        i2c->busy = false;
                        transaction_finished = true;
                    }
                }
            }
            break;

        default:
            break;
    }

    if(transaction_finished && i2c->callback != NULL) {
        i2c->callback(i2c);
    }
}

static void I2Cx_ER_IRQ_execute(int slot) {
    assert(slot >= 0);

    i2c_t *i2c = &i2c_pool[slot];
    uint32_t sr1 = i2c->res->reg->SR1;

    if (sr1 & I2C_SR1_AF) {
        i2c->res->reg->SR1 = (uint32_t)~I2C_SR1_AF; // reset af
        if(i2c->retry_cnt < i2c->current_ctx->max_retries) { // restart
            i2c->retry_cnt++;
            i2c->res->reg->CR1 |= I2C_CR1_START;
            i2c->state = I2C_START;
            return;
        }
        i2c->state = I2C_ERROR;
        i2c->status = BMML_ERROR;
        i2c->hw_error = I2C_SR1_AF; // Acknowledge Failure
    }
    else if (sr1 & I2C_SR1_BERR) {
        i2c->state = I2C_ERROR;
        i2c->res->reg->SR1 = (uint32_t)~I2C_SR1_BERR;
        i2c->status = BMML_ERROR;
        i2c->hw_error = I2C_SR1_BERR;   // BUS ERROR
    }

    if(i2c->xfer_mode == I2C_XFER_DMA) {
        i2c->res->reg->CR2 &= ~I2C_CR2_DMAEN;
        if(i2c->dma_tx_acquired) {
            i2c->dma_tx.stream->CR &= ~DMA_SxCR_EN;
            while(i2c->dma_tx.stream->CR & DMA_SxCR_EN) { }
        }
        if(i2c->dma_rx_acquired) {
            i2c->dma_rx.stream->CR &= ~DMA_SxCR_EN;
            while(i2c->dma_rx.stream->CR & DMA_SxCR_EN) { }
        }
    }

    i2c->res->reg->CR1 |= I2C_CR1_STOP;

    for(volatile uint32_t i = 0; i < 50; i++);

    i2c->res->reg->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    
    i2c->busy = false;

    if (i2c->callback != NULL) {
        i2c->callback(i2c);
    }
}


// Event interrupt handlers

void I2C1_EV_IRQHandler(void) {
    if(!i2c_taken[0]) return;
    I2Cx_EV_IRQ_execute(0);
}

void I2C2_EV_IRQHandler(void) {
    if(!i2c_taken[1]) return;
    I2Cx_EV_IRQ_execute(1);
}

void I2C3_EV_IRQHandler(void) {
    if(!i2c_taken[2]) return;
    I2Cx_EV_IRQ_execute(2);
}

// Error interrupt handlers

void I2C1_ER_IRQHandler(void) {
    if(!i2c_taken[0]) return;
    I2Cx_ER_IRQ_execute(0);
}

void I2C2_ER_IRQHandler(void) {
    if(!i2c_taken[1]) return;
    I2Cx_ER_IRQ_execute(1);
}

void I2C3_ER_IRQHandler(void) {
    if(!i2c_taken[2]) return;
    I2Cx_ER_IRQ_execute(2); 
}

static void DMA_finish(i2c_t *i2c) {
    i2c->res->reg->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    i2c->res->reg->CR1 |= I2C_CR1_STOP;
    i2c->state  = I2C_DONE;
    i2c->status = BMML_OK;
    i2c->busy   = false;
}

static void DMA_tx_complete(i2c_t *i2c) {
    if (i2c->ctx.repeated_start && i2c->ctx.rx_len > 0) {
        i2c->res->reg->CR2 &= ~I2C_CR2_DMAEN;
        i2c->ctx.tx_len = 0;
        i2c->state = I2C_START;
        i2c->res->reg->CR1 |= I2C_CR1_START;
        return;
    }

    DMA_finish(i2c);
}

static void DMA_rx_complete(i2c_t *i2c) {
    DMA_finish(i2c);
}

static inline uint32_t dma_teif_mask(uint32_t tcif_mask) {
    return tcif_mask >> 2;
}

static bool dma_check_flag(dma_t *d, uint32_t mask, bool clear) {
    volatile uint32_t *isr  = (d->num_stream <= 3) ? &d->res->reg->LISR : &d->res->reg->HISR;
    volatile uint32_t *ifcr = (d->num_stream <= 3) ? &d->res->reg->LIFCR : &d->res->reg->HIFCR;
    if (*isr & mask) {
        if (clear) *ifcr = mask;
        return true;
    }
    return false;
}

static void DMA_error(i2c_t *i2c, dma_t *d, uint32_t teif_mask) {
    dma_check_flag(d, teif_mask, true); 
    d->stream->CR &= ~DMA_SxCR_EN; 
    while (d->stream->CR & DMA_SxCR_EN) { } 
    i2c->res->reg->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    i2c->res->reg->CR1 |= I2C_CR1_STOP;
    i2c->state   = I2C_ERROR;
    i2c->status  = BMML_ERROR;
    i2c->hw_error = teif_mask;
    i2c->busy    = false;
}


static void DMA1_StreamX_IRQHandler(i2c_t *i2c) {
    if(i2c->dma_tx_acquired) {
        uint32_t teif = dma_teif_mask(i2c->res->tx_tcif_mask);
        if(dma_check_flag(&i2c->dma_tx, teif, false)) {
            DMA_error(i2c, &i2c->dma_tx, teif);
            if(i2c->callback != NULL) i2c->callback(i2c);
            return;
        }
        if(dma_check_flag(&i2c->dma_tx, i2c->res->tx_tcif_mask, true)) {
            DMA_tx_complete(i2c);
            if(i2c->callback != NULL) i2c->callback(i2c);
            return;
        }
    }
    if(i2c->dma_rx_acquired) {
        uint32_t teif = dma_teif_mask(i2c->res->rx_tcif_mask);
        if(dma_check_flag(&i2c->dma_rx, teif, false)) {
            DMA_error(i2c, &i2c->dma_rx, teif);
            if(i2c->callback != NULL) i2c->callback(i2c);
            return;
        }
        if(dma_check_flag(&i2c->dma_rx, i2c->res->rx_tcif_mask, true)) {
            DMA_rx_complete(i2c);
            if(i2c->callback != NULL) i2c->callback(i2c);
            return;
        }
    }
}

// I2C1 TX
void DMA1_Stream6_IRQHandler(void) {
    if(!i2c_taken[0]) return;
    DMA1_StreamX_IRQHandler(&i2c_pool[0]);
}

// I2C1 RX
void DMA1_Stream0_IRQHandler(void) {  
    if(!i2c_taken[0]) return;
    DMA1_StreamX_IRQHandler(&i2c_pool[0]);
}


// I2C2 TX
void DMA1_Stream7_IRQHandler(void) {
    if(!i2c_taken[1]) return;
    DMA1_StreamX_IRQHandler(&i2c_pool[1]);

}

// I2C2 and I2C3 RX
void DMA1_Stream2_IRQHandler(void) {
    if(i2c_taken[1] && i2c_pool[1].dma_rx_acquired) {
        DMA1_StreamX_IRQHandler(&i2c_pool[1]);
    } else if(i2c_taken[2] && i2c_pool[2].dma_rx_acquired) {
        DMA1_StreamX_IRQHandler(&i2c_pool[2]);
    }
}

// I2C3 TX
void DMA1_Stream4_IRQHandler(void) {
    if(!i2c_taken[2]) return;
    DMA1_StreamX_IRQHandler(&i2c_pool[2]);
}



