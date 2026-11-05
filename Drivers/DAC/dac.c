#include "dac.h"
#include "bmml_dma_shared.h"
#include "bmml_res_dac.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include "timer.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define DAC_COUNT           1
#define DAC_DMA_STREAM      5       /* DAC ch1: DMA1 Stream5, channel 7 */
#define DAC_DMA_CHANNEL     7U
#define DAC_CH1_CR_MASK     0x0000FFFFU

static dac_t dac_pool[DAC_COUNT];
static bool  dac_taken[DAC_COUNT];

static int dac_slot(const DAC_TypeDef *reg) {
    return (reg == DAC1) ? 0 : -1;
}

bmml_status_t dac_dma_acquire(DAC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream,
                              uint16_t *wave_buffer, uint16_t num_points,
                              uint32_t sample_rate_hz, dac_t **out) {
    if(out) *out = NULL;
    if(reg == NULL || dma == NULL) return BMML_INVALID_ARG;
    if(dma != DMA1 || stream != DAC_DMA_STREAM) return BMML_INVALID_ARG;
    if(wave_buffer == NULL || num_points == 0) return BMML_INVALID_ARG;

    int slot = dac_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(dac_taken[slot]) return BMML_BUSY;

    int idx = dac_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;
    int dma_idx = dma_res_idx(dma);
    if(dma_idx < 0) return BMML_INVALID_ARG;

    // 1. Timer first: it validates the frequency and has no side effects on failure
    timer_basic_t *timer;
    bmml_status_t st = timer_basic_acquire_trigger(TIM6, sample_rate_hz, &timer);
    if(st != BMML_OK) return st;

    // 2. DMA stream
    DMA_Stream_TypeDef *dma_stream;
    st = bmml_dma_acquire_stream(dma, stream, NULL, NULL, &dma_stream);
    if(st != BMML_OK) {
        timer_basic_release(timer);
        return st;
    }

    // 3. Fill the handle
    dac_taken[slot] = true;
    dac_t *dac = &dac_pool[slot];
    *dac = (dac_t){0};
    dac->res            = &dac_res[idx];
    dac->wave_buffer    = wave_buffer;
    dac->num_points     = num_points;
    dac->sample_rate_hz = sample_rate_hz;
    dac->timer          = timer;
    dac->dma.res        = &dma_res[dma_idx];
    dac->dma.stream     = dma_stream;
    dac->dma.num_stream = stream;
    dac->dma.channel    = DAC_DMA_CHANNEL;

    // 4. Clocks
    *dac->res->rcc_reg |= dac->res->rcc_mask;
    *dac->dma.res->rcc_reg |= dac->dma.res->rcc_mask;
    (void)*dac->res->rcc_reg;

    // 5. DMA static config (addresses/NDTR are written in dac_start)
    dma_disable_stream(dac->dma.stream);
    dma_clear_stream_flags(dac->dma.res->reg, dac->dma.num_stream);

    uint32_t cr = 0U;
    cr |= (dac->dma.channel << DMA_SxCR_CHSEL_Pos);
    cr |= DMA_SxCR_MINC;                    // memory increment
    cr |= (1U << DMA_SxCR_DIR_Pos);         // memory -> peripheral
    cr |= DMA_SxCR_CIRC;
    cr |= (1U << DMA_SxCR_MSIZE_Pos);       // 16 bit
    cr |= (1U << DMA_SxCR_PSIZE_Pos);       // 16 bit
    dac->dma.stream->CR = cr;

    // 6. TIM6 -> TRGO on update event (MMS = 010)
    dac->timer->res->reg->CR2 = (dac->timer->res->reg->CR2 & ~TIM_CR2_MMS)
                              | (2U << TIM_CR2_MMS_Pos);

    // 7. DAC ch1: trigger enabled, TSEL1 = 000 (TIM6 TRGO), DMA, then enable
    dac->res->reg->CR &= ~DAC_CH1_CR_MASK;
    dac->res->reg->CR |= DAC_CR_TEN1 | DAC_CR_DMAEN1;
    dac->res->reg->CR |= DAC_CR_EN1;

    if(out) *out = dac;
    return BMML_OK;
}

bmml_status_t dac_start(dac_t *dac) {
    if(dac == NULL || dac->res == NULL) return BMML_INVALID_ARG;
    if(dac->running) return BMML_OK;

    // Stream must be off before touching NDTR/M0AR
    if(dma_disable_stream(dac->dma.stream) != BMML_OK) return BMML_TIMEOUT;
    dma_clear_stream_flags(dac->dma.res->reg, dac->dma.num_stream);

    // Clear a possible underrun and re-arm DMA requests of the DAC
    dac->res->reg->SR = DAC_SR_DMAUDR1;
    dac->res->reg->CR &= ~DAC_CR_DMAEN1;
    dac->res->reg->CR |= DAC_CR_DMAEN1;

    // Re-load every time: after a stop NDTR/M0AR are not at the buffer start
    dac->dma.stream->PAR  = (uint32_t)&dac->res->reg->DHR12R1;
    dac->dma.stream->M0AR = (uint32_t)dac->wave_buffer;
    dac->dma.stream->NDTR = dac->num_points;

    dac->dma.stream->CR |= DMA_SxCR_EN;     // stream first...
    timer_basic_start(dac->timer);          // ...then triggers
    dac->running = true;
    return BMML_OK;
}

bmml_status_t dac_stop(dac_t *dac) {
    if(dac == NULL || dac->res == NULL) return BMML_INVALID_ARG;
    if(!dac->running) return BMML_OK;

    timer_basic_stop(dac->timer);           // triggers first...
    bmml_status_t st = dma_disable_stream(dac->dma.stream);   // ...then DMA
    dma_clear_stream_flags(dac->dma.res->reg, dac->dma.num_stream);
    dac->running = false;
    return st;
}

bmml_status_t dac_set_sample_rate(dac_t *dac, uint32_t sample_rate_hz) {
    if(dac == NULL || dac->res == NULL) return BMML_INVALID_ARG;
    bmml_status_t st = timer_basic_set_freq_hz(dac->timer, sample_rate_hz);
    if(st == BMML_OK) dac->sample_rate_hz = sample_rate_hz;
    return st;
}

bmml_status_t dac_dma_release(dac_t *dac) {
    if(dac == NULL || dac->res == NULL) return BMML_INVALID_ARG;

    int slot = dac_slot(dac->res->reg);
    if(slot < 0 || !dac_taken[slot]) return BMML_INVALID_ARG;

    // Stop the source of requests, then the consumer
    if(dac_stop(dac) != BMML_OK) return BMML_TIMEOUT;
    dac->res->reg->CR &= ~(DAC_CR_EN1 | DAC_CR_DMAEN1 | DAC_CR_TEN1);

    bmml_dma_release_stream(dac->dma.res->reg, dac->dma.num_stream);

    // MMS must be cleared BEFORE timer_basic_release (it zeroes the handle)
    TIM_TypeDef *tim = dac->timer->res->reg;
    tim->CR2 &= ~TIM_CR2_MMS;
    timer_basic_release(dac->timer);

    dac_taken[slot] = false;
    dac_pool[slot] = (dac_t){0};
    return BMML_OK;
}