#include "i2c.h"
#include "bmml_dma_shared.h"
#include "bmml_res_i2c.h"
#include "bmml_status.h"
#include "bmml_utilities.h"
#include "bmml_irq_shared.h"
#include "stm32f446xx.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
 
#define I2C_COUNT               3
#define I2C_IRQ_PRIORITY        BMML_IRQ_PRIORITY
#define I2C_STOP_WAIT_LOOPS     20000U
 
#define I2C_ERR_MASK  (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | \
                       I2C_SR1_OVR  | I2C_SR1_PECERR | I2C_SR1_TIMEOUT)
/* SR1 error flags are rc_w0: write 0 to clear, 1 to leave untouched */
#define I2C_SR1_KEEP_EXCEPT(flags)  ((~(uint32_t)(flags)) & 0xFFFFU)
 
static void i2c_dma_tx_isr(void *ctx);
static void i2c_dma_rx_isr(void *ctx);
 
static i2c_t i2c_pool[I2C_COUNT];
static volatile bool i2c_taken[I2C_COUNT];
 
static int i2c_slot(const I2C_TypeDef *reg) {
    if(reg == I2C1) return 0;
    if(reg == I2C2) return 1;
    if(reg == I2C3) return 2;
    return -1;
}
 
/* ------------------------------------------------------------------------- */
/*  Helpers                                                                  */
/* ------------------------------------------------------------------------- */
 
static void i2c_reset_progress(i2c_t *i2c) {
    i2c->tx_cnt   = 0U;
    i2c->rx_cnt   = 0U;
    i2c->rx_phase = (i2c->ctx.tx_len == 0U);   /* read-only transaction starts in RX phase */
}
 
static void i2c_dma_stop(dma_t *d) {
    (void)dma_disable_stream(d->stream);       /* bounded wait, safe in ISR */
    dma_clear_stream_flags(d->res->reg, d->num_stream);
}
 
/* Acquire the DMA stream for one direction (lazy, kept until i2c_release). */
static bmml_status_t i2c_dma_acquire_dir(i2c_t *i2c, bool tx) {
    bool *acquired = tx ? &i2c->dma_tx_acquired : &i2c->dma_rx_acquired;
    if(*acquired) return BMML_OK;
 
    dma_t *d = tx ? &i2c->dma_tx : &i2c->dma_rx;
    uint8_t  stream  = tx ? i2c->res->dma_tx_stream  : i2c->res->dma_rx_stream;
    uint32_t channel = tx ? i2c->res->dma_tx_channel : i2c->res->dma_rx_channel;
 
    int dma_idx = dma_res_idx(DMA1);           /* all I2C requests are on DMA1 */
    if(dma_idx < 0) return BMML_INVALID_ARG;
 
    DMA_Stream_TypeDef *s;
    bmml_status_t st = bmml_dma_acquire_stream(DMA1, stream,
                           tx ? i2c_dma_tx_isr : i2c_dma_rx_isr, i2c, &s);
    if(st != BMML_OK) return st;
 
    *acquired    = true;
    d->res       = &dma_res[dma_idx];
    d->stream    = s;
    d->num_stream = stream;
    d->channel   = channel;
 
    *d->res->rcc_reg |= d->res->rcc_mask;
    (void)*d->res->rcc_reg;                    /* RCC write-to-effect delay */
 
    dma_disable_stream(s);
    dma_clear_stream_flags(d->res->reg, stream);
    return BMML_OK;
}
 
/* Program one DMA stream for a phase. Stream is NOT enabled here. */
static bmml_status_t i2c_dma_arm(i2c_t *i2c, dma_t *d, bool tx, const uint8_t *buf, size_t len) {
    if(dma_disable_stream(d->stream) != BMML_OK) return BMML_TIMEOUT;
    dma_clear_stream_flags(d->res->reg, d->num_stream);
 
    d->stream->PAR  = (uint32_t)&i2c->res->reg->DR;
    d->stream->M0AR = (uint32_t)buf;
    d->stream->NDTR = (uint32_t)len;
 
    uint32_t cr = (d->channel << DMA_SxCR_CHSEL_Pos)
                | DMA_SxCR_MINC
                | DMA_SxCR_TCIE
                | DMA_SxCR_TEIE;
    if(tx) cr |= DMA_SxCR_DIR_0;               /* memory -> peripheral */
    d->stream->CR = cr;
    return BMML_OK;
}
 
/*
 * Single exit point of every transaction (success or failure).
 * Must be the LAST thing done with the handle: the callback may start a new transfer.
 * CR1 is only touched for STOP: START/STOP are cleared by hardware, so a
 * read-modify-write of CR1 after setting them could re-arm them.
 */
static void i2c_finish(i2c_t *i2c, bmml_status_t st, uint32_t hw_err, bool send_stop) {
    I2C_TypeDef *r = i2c->res->reg;
 
    r->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN |
                I2C_CR2_DMAEN   | I2C_CR2_LAST);
    if(send_stop) r->CR1 |= I2C_CR1_STOP;
 
    if(i2c->dma_tx_acquired) i2c_dma_stop(&i2c->dma_tx);
    if(i2c->dma_rx_acquired) i2c_dma_stop(&i2c->dma_rx);
 
    i2c->state    = (st == BMML_OK) ? I2C_DONE : I2C_ERROR;
    i2c->status   = st;
    i2c->hw_error = hw_err;
    i2c->busy     = false;
 
    if(i2c->callback != NULL) i2c->callback(i2c);
}
 
/* ------------------------------------------------------------------------- */
/*  Acquire / release                                                        */
/* ------------------------------------------------------------------------- */
 
bmml_status_t i2c_acquire(I2C_TypeDef *reg, i2c_mode_t mode, i2c_callback_t cb, i2c_t **out) {
    if(out) *out = NULL;
    if(reg == NULL) return BMML_INVALID_ARG;
    if(mode != I2C_SM_100KHZ && mode != I2C_FM_400KHZ) return BMML_INVALID_ARG;
 
    int slot = i2c_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(i2c_taken[slot]) return BMML_BUSY;
 
    int idx = i2c_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;
 
    /* Validate and compute timing BEFORE claiming the slot (no leak on error) */
    const uint32_t pclk     = get_apb1_clock_hz();
    const uint32_t freq_mhz = pclk / 1000000U;
    const uint32_t min_mhz  = (mode == I2C_SM_100KHZ) ? 2U : 4U;
    if(freq_mhz < min_mhz || freq_mhz > 50U) return BMML_INVALID_ARG;
 
    uint32_t ccr, trise;
    if(mode == I2C_SM_100KHZ) {
        /* Thigh = Tlow = CCR * Tpclk -> Fscl = pclk / (2 * CCR). Round UP: never exceed 100 kHz */
        ccr = (pclk + 2U * 100000U - 1U) / (2U * 100000U);
        if(ccr < 4U) ccr = 4U;
        trise = freq_mhz + 1U;                         /* 1000 ns max rise time */
    } else {
        /* DUTY = 0: Tlow = 2 * Thigh -> Fscl = pclk / (3 * CCR). Round UP: never exceed 400 kHz */
        ccr = (pclk + 3U * 400000U - 1U) / (3U * 400000U);
        if(ccr < 1U) ccr = 1U;
        trise = (freq_mhz * 300U) / 1000U + 1U;        /* 300 ns max rise time */
    }
    if(ccr > 0xFFFU) return BMML_INVALID_ARG;
 
    i2c_taken[slot] = true;
    i2c_t *i2c = &i2c_pool[slot];
    *i2c = (i2c_t){0};
    i2c->res      = &i2c_res[idx];
    i2c->callback = cb;
    i2c->state    = I2C_IDLE;
 
    *i2c->res->rcc_reg |= i2c->res->rcc_mask;
    (void)*i2c->res->rcc_reg;
 
    /* Clean state: PE = 0, software reset pulse */
    reg->CR1 = 0U;
    reg->CR1 = I2C_CR1_SWRST;
    reg->CR1 = 0U;
 
    reg->CR2   = (freq_mhz & I2C_CR2_FREQ);
    reg->CCR   = ((mode == I2C_FM_400KHZ) ? I2C_CCR_FS : 0U) | (ccr & I2C_CCR_CCR);
    reg->TRISE = (trise & I2C_TRISE_TRISE);
 
    NVIC_SetPriority(i2c->res->irqn_ev, I2C_IRQ_PRIORITY);
    NVIC_SetPriority(i2c->res->irqn_er, I2C_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(i2c->res->irqn_ev);
    NVIC_ClearPendingIRQ(i2c->res->irqn_er);
    NVIC_EnableIRQ(i2c->res->irqn_ev);
    NVIC_EnableIRQ(i2c->res->irqn_er);
 
    i2c_enable(reg);
 
    if(out) *out = i2c;
    return BMML_OK;
}
 
bmml_status_t i2c_release(i2c_t *i2c) {
    if(i2c == NULL || i2c->res == NULL) return BMML_INVALID_ARG;
    if(i2c->busy) return BMML_BUSY;
 
    I2C_TypeDef *r = i2c->res->reg;
    int slot = i2c_slot(r);
    if(slot < 0 || !i2c_taken[slot]) return BMML_INVALID_ARG;
 
    NVIC_DisableIRQ(i2c->res->irqn_ev);
    NVIC_DisableIRQ(i2c->res->irqn_er);
 
    i2c_reset_off(r);
    r->CR2   = 0U;
    r->CCR   = 0U;
    r->TRISE = 0x0002U;                                /* reset value */
 
    if(i2c->dma_tx_acquired) {
        i2c_dma_stop(&i2c->dma_tx);
        bmml_dma_release_stream(i2c->dma_tx.res->reg, i2c->dma_tx.num_stream);
    }
    if(i2c->dma_rx_acquired) {
        i2c_dma_stop(&i2c->dma_rx);
        bmml_dma_release_stream(i2c->dma_rx.res->reg, i2c->dma_rx.num_stream);
    }
 
    *i2c->res->rcc_reg &= ~i2c->res->rcc_mask;
 
    i2c_taken[slot] = false;
    i2c_pool[slot]  = (i2c_t){0};
    return BMML_OK;
}
 
/* ------------------------------------------------------------------------- */
/*  Transfer start                                                           */
/* ------------------------------------------------------------------------- */
 
static bmml_status_t i2c_start_xfer(i2c_t *i2c, const i2c_transaction_t *tr, i2c_xfer_mode_t mode) {
    if(i2c == NULL || i2c->res == NULL || tr == NULL) return BMML_INVALID_ARG;
    if(i2c->busy) return BMML_BUSY;
 
    if(tr->addr > 0x7FU) return BMML_INVALID_ARG;
    if(tr->tx_len == 0U && tr->rx_len == 0U) return BMML_INVALID_ARG;
    if(tr->tx_len > 0U && tr->tx_buff == NULL) return BMML_INVALID_ARG;
    if(tr->rx_len > 0U && tr->rx_buff == NULL) return BMML_INVALID_ARG;
    /* write + read is always done with a repeated START */
    if(tr->tx_len > 0U && tr->rx_len > 0U && !tr->repeated_start) return BMML_INVALID_ARG;
 
    if(mode == I2C_XFER_DMA) {
        if(tr->tx_len > 0xFFFFU || tr->rx_len > 0xFFFFU) return BMML_INVALID_ARG;   /* NDTR is 16 bit */
        bmml_status_t st;
        if(tr->tx_len > 0U && (st = i2c_dma_acquire_dir(i2c, true))  != BMML_OK) return st;
        if(tr->rx_len > 0U && (st = i2c_dma_acquire_dir(i2c, false)) != BMML_OK) return st;
    }
 
    I2C_TypeDef *r = i2c->res->reg;
 
    /* The STOP of a previous transaction (possibly requested just before the callback)
       must be generated before we can issue a new START. */
    uint32_t guard = I2C_STOP_WAIT_LOOPS;
    while((r->CR1 & I2C_CR1_STOP) && guard > 0U) guard--;
    if(guard == 0U) return BMML_BUSY;
 
    /* Claim the handle atomically */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(i2c->busy) {
        __set_PRIMASK(primask);
        return BMML_BUSY;
    }
    i2c->busy = true;
    __set_PRIMASK(primask);
 
    i2c->xfer_mode = mode;
    i2c->ctx       = *tr;
    i2c->retry_cnt = 0U;
    i2c->status    = BMML_OK;
    i2c->hw_error  = 0U;
    i2c_reset_progress(i2c);
 
    r->CR1 &= ~I2C_CR1_POS;
    r->CR1 |= I2C_CR1_ACK;
    r->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST | I2C_CR2_ITBUFEN);
    r->SR1 = I2C_SR1_KEEP_EXCEPT(I2C_ERR_MASK);        /* drop stale error flags */
 
    i2c->state = I2C_START;
    r->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    r->CR1 |= I2C_CR1_START;
    return BMML_OK;
}
 
bmml_status_t i2c_it_transmit(i2c_t *i2c, i2c_transaction_t *tr) {
    return i2c_start_xfer(i2c, tr, I2C_XFER_IT);
}
 
bmml_status_t i2c_dma_transmit(i2c_t *i2c, i2c_transaction_t *tr) {
    return i2c_start_xfer(i2c, tr, I2C_XFER_DMA);
}
 
bmml_status_t i2c_dma_receive(i2c_t *i2c, i2c_transaction_t *tr) {
    if(tr == NULL || tr->tx_len != 0U || tr->rx_len == 0U) return BMML_INVALID_ARG;
    return i2c_start_xfer(i2c, tr, I2C_XFER_DMA);
}
 
/* ------------------------------------------------------------------------- */
/*  Event ISR (SB, ADDR, TXE, RXNE, BTF)                                     */
/* ------------------------------------------------------------------------- */
 
static void i2c_ev_handler(int slot) {
    i2c_t *i2c = &i2c_pool[slot];
    I2C_TypeDef *r = i2c->res->reg;
    uint32_t sr1 = r->SR1;
 
    switch(i2c->state) {
 
    case I2C_START:
        if(sr1 & I2C_SR1_SB) {
            /* SB is cleared by: read SR1 (done above) + write DR */
            r->DR = (uint8_t)((i2c->ctx.addr << 1) | (i2c->rx_phase ? 1U : 0U));
            i2c->state = I2C_ADDR;
        }
        break;
 
    case I2C_ADDR: {
        if(!(sr1 & I2C_SR1_ADDR)) break;
        const bool dma = (i2c->xfer_mode == I2C_XFER_DMA);
 
        if(!i2c->rx_phase) {
            /* ---------------- master transmitter ---------------- */
            if(dma) {
                if(i2c_dma_arm(i2c, &i2c->dma_tx, true, i2c->ctx.tx_buff, i2c->ctx.tx_len) != BMML_OK) {
                    (void)r->SR2;                       /* release SCL stretch */
                    i2c_finish(i2c, BMML_TIMEOUT, 0U, true);
                    return;
                }
                r->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);   /* quiet while DMA works */
                r->CR2 |= I2C_CR2_DMAEN;
                i2c->dma_tx.stream->CR |= DMA_SxCR_EN;
            } else {
                r->CR2 |= I2C_CR2_ITBUFEN;              /* TXE is already 1 -> first byte in TX state */
            }
            i2c->state = I2C_TX;
            (void)r->SR2;                               /* clear ADDR: SR1 read + SR2 read */
        } else {
            /* ---------------- master receiver ---------------- */
            const size_t n = i2c->ctx.rx_len;
 
            if(dma) {
                if(i2c_dma_arm(i2c, &i2c->dma_rx, false, i2c->ctx.rx_buff, n) != BMML_OK) {
                    (void)r->SR2;
                    i2c_finish(i2c, BMML_TIMEOUT, 0U, true);
                    return;
                }
                if(n == 1U) {
                    r->CR1 &= ~I2C_CR1_ACK;             /* NACK must be armed BEFORE clearing ADDR */
                } else {
                    r->CR1 |= I2C_CR1_ACK;
                    r->CR2 |= I2C_CR2_LAST;             /* DMA EOT -> NACK on the last byte */
                }
                r->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
                r->CR2 |= I2C_CR2_DMAEN;
                i2c->dma_rx.stream->CR |= DMA_SxCR_EN;
                i2c->state = I2C_RX;
                (void)r->SR2;
                if(n == 1U) r->CR1 |= I2C_CR1_STOP;     /* STOP only AFTER clearing ADDR */
            } else {
                i2c->state = I2C_RX;
                if(n == 1U) {
                    r->CR1 &= ~I2C_CR1_ACK;             /* 1 byte: ACK off BEFORE clearing ADDR */
                    r->CR2 |= I2C_CR2_ITBUFEN;
                    (void)r->SR2;
                    r->CR1 |= I2C_CR1_STOP;             /* ... STOP right after */
                } else if(n == 2U) {
                    r->CR1 &= ~I2C_CR1_ACK;             /* 2 bytes: POS = 1, ACK = 0 -> NACK hits 2nd byte */
                    r->CR1 |= I2C_CR1_POS;
                    r->CR2 &= ~I2C_CR2_ITBUFEN;         /* only BTF is interesting */
                    (void)r->SR2;
                } else {
                    r->CR1 |= I2C_CR1_ACK;
                    if(n > 3U) r->CR2 |= I2C_CR2_ITBUFEN;
                    else       r->CR2 &= ~I2C_CR2_ITBUFEN;   /* n == 3 starts directly at the BTF stage */
                    (void)r->SR2;
                }
            }
        }
        break;
    }
 
    case I2C_TX:
        if(i2c->xfer_mode == I2C_XFER_IT &&
           (sr1 & I2C_SR1_TXE) && i2c->tx_cnt < i2c->ctx.tx_len) {
            r->DR = i2c->ctx.tx_buff[i2c->tx_cnt++];
            if(i2c->tx_cnt >= i2c->ctx.tx_len) {
                r->CR2 &= ~I2C_CR2_ITBUFEN;             /* last byte loaded, wait for BTF only */
            }
            break;
        }
        /* BTF with everything already handed to DR: the last byte is on the wire.
           (DMA: tx_cnt is set to tx_len by the DMA TC ISR.) */
        if((sr1 & I2C_SR1_BTF) && i2c->tx_cnt >= i2c->ctx.tx_len) {
            if(i2c->ctx.rx_len > 0U) {
                r->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_ITBUFEN);
                i2c->rx_phase = true;
                i2c->state = I2C_START;
                r->CR2 |= I2C_CR2_ITEVTEN;
                r->CR1 |= I2C_CR1_START;                /* repeated START */
            } else {
                i2c_finish(i2c, BMML_OK, 0U, true);     /* STOP */
                return;
            }
        }
        break;
 
    case I2C_RX:
        if(i2c->xfer_mode != I2C_XFER_IT) break;        /* DMA RX completes via the DMA TC ISR */
        {
            uint8_t *buf = i2c->ctx.rx_buff;
            size_t rem = i2c->ctx.rx_len - i2c->rx_cnt;
 
            if(rem > 3U) {
                if(sr1 & I2C_SR1_RXNE) {
                    buf[i2c->rx_cnt++] = (uint8_t)r->DR;
                    if(--rem == 3U) r->CR2 &= ~I2C_CR2_ITBUFEN;   /* from here on only BTF */
                }
            } else if(rem == 3U) {
                /* BTF: byte N-2 in DR, N-1 in shift register, SCL stretched */
                if(sr1 & I2C_SR1_BTF) {
                    r->CR1 &= ~I2C_CR1_ACK;             /* NACK for the last byte */
                    buf[i2c->rx_cnt++] = (uint8_t)r->DR;
                }
            } else if(rem == 2U) {
                /* BTF: byte N-1 in DR, N in shift register, SCL stretched */
                if(sr1 & I2C_SR1_BTF) {
                    r->CR1 |= I2C_CR1_STOP;
                    buf[i2c->rx_cnt++] = (uint8_t)r->DR;
                    buf[i2c->rx_cnt++] = (uint8_t)r->DR;
                    i2c_finish(i2c, BMML_OK, 0U, false);    /* STOP already requested */
                    return;
                }
            } else if(rem == 1U) {
                if(sr1 & I2C_SR1_RXNE) {                /* STOP was requested in the ADDR stage */
                    buf[i2c->rx_cnt++] = (uint8_t)r->DR;
                    i2c_finish(i2c, BMML_OK, 0U, false);
                    return;
                }
            }
        }
        break;
 
    default:
        break;
    }
}
 
/* ------------------------------------------------------------------------- */
/*  Error ISR                                                                */
/* ------------------------------------------------------------------------- */
 
static void i2c_er_handler(int slot) {
    i2c_t *i2c = &i2c_pool[slot];
    I2C_TypeDef *r = i2c->res->reg;
 
    uint32_t err = r->SR1 & I2C_ERR_MASK;
    if(err == 0U) return;
    r->SR1 = I2C_SR1_KEEP_EXCEPT(err);                  /* clear what we are about to handle */
 
    if(!i2c->busy) return;
 
    /* NACK: restart the whole transaction with a repeated START (RM: STOP or Sr is allowed after AF) */
    if((err & I2C_SR1_AF) && i2c->retry_cnt < i2c->ctx.max_retries) {
        i2c->retry_cnt++;
        if(i2c->dma_tx_acquired) i2c_dma_stop(&i2c->dma_tx);
        if(i2c->dma_rx_acquired) i2c_dma_stop(&i2c->dma_rx);
        r->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST | I2C_CR2_ITBUFEN);
        r->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
        r->CR1 &= ~I2C_CR1_POS;
        r->CR1 |= I2C_CR1_ACK;
        i2c_reset_progress(i2c);
        i2c->state = I2C_START;
        r->CR1 |= I2C_CR1_START;
        return;
    }
 
    /* After ARLO the peripheral already left master mode: no STOP */
    i2c_finish(i2c, BMML_ERROR, err, !(err & I2C_SR1_ARLO));
}
 
/* ------------------------------------------------------------------------- */
/*  DMA ISRs                                                                 */
/* ------------------------------------------------------------------------- */
 
static void i2c_dma_tx_isr(void *ctx) {
    i2c_t *i2c = (i2c_t *)ctx;
    const uint32_t teif = dma_teif_mask(i2c->res->tx_tcif_mask);
 
    if(dma_check_flag(&i2c->dma_tx, teif, true)) {
        i2c_finish(i2c, BMML_ERROR, teif, true);
        return;
    }
    if(dma_check_flag(&i2c->dma_tx, i2c->res->tx_tcif_mask, true)) {
        /* Last byte went into DR, but is not on the wire yet:
           let BTF (event IRQ) tell us when it is safe to STOP / Sr. */
        i2c->tx_cnt = i2c->ctx.tx_len;
        i2c->res->reg->CR2 |= I2C_CR2_ITEVTEN;
    }
}
 
static void i2c_dma_rx_isr(void *ctx) {
    i2c_t *i2c = (i2c_t *)ctx;
    const uint32_t teif = dma_teif_mask(i2c->res->rx_tcif_mask);
 
    if(dma_check_flag(&i2c->dma_rx, teif, true)) {
        i2c_finish(i2c, BMML_ERROR, teif, true);
        return;
    }
    if(dma_check_flag(&i2c->dma_rx, i2c->res->rx_tcif_mask, true)) {
        i2c->rx_cnt = i2c->ctx.rx_len;
        /* 1-byte read already requested STOP in the ADDR stage */
        i2c_finish(i2c, BMML_OK, 0U, i2c->ctx.rx_len != 1U);
    }
}
 
/* ------------------------------------------------------------------------- */
/*  Vectors                                                                  */
/* ------------------------------------------------------------------------- */
 
void I2C1_EV_IRQHandler(void) { if(i2c_taken[0]) i2c_ev_handler(0); }
void I2C2_EV_IRQHandler(void) { if(i2c_taken[1]) i2c_ev_handler(1); }
void I2C3_EV_IRQHandler(void) { if(i2c_taken[2]) i2c_ev_handler(2); }
 
void I2C1_ER_IRQHandler(void) { if(i2c_taken[0]) i2c_er_handler(0); }
void I2C2_ER_IRQHandler(void) { if(i2c_taken[1]) i2c_er_handler(1); }
void I2C3_ER_IRQHandler(void) { if(i2c_taken[2]) i2c_er_handler(2); }
 