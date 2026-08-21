#include "dac.h"
#include "bmml_dma_shared.h"
#include "bmml_res_dac.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include "timer.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


#define DAC_COUNT 1
#define DMA_DEFAULT_CHANNEL 7


static dac_t dac_pool[DAC_COUNT];
static bool dac_taken[DAC_COUNT];

static int dac_slot(const DAC_TypeDef *reg) {
    if(reg == DAC1) return 0;
    return -1;
}


bmml_status_t dac_dma_acquire(DAC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream, uint16_t *wave_buffer, 
    uint16_t num_points, dac_t **out) {
    if(out) *out = NULL;
    if(reg == NULL || dma == NULL) return BMML_INVALID_ARG;
    if(reg == DAC1 && (dma != DMA1 || stream !=5)) return BMML_INVALID_ARG;

    int slot = dac_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(dac_taken[slot]) return BMML_BUSY;

    int idx = dac_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    int dma_idx = dma_res_idx(dma);
    if(dma_idx < 0) return BMML_INVALID_ARG;
    
    // Acquire DMA stream
    DMA_Stream_TypeDef *dma_stream;
    bmml_status_t st = bmml_dma_acquire_stream(dma, stream, NULL, NULL, &dma_stream);
    if(st != BMML_OK) return st;
    // Acquire TIM6 timer
    timer_basic_t *timer;
    bmml_status_t tmr_status = timer_basic_acquire(TIM6, 100, NULL, &timer);
    if(tmr_status != BMML_OK) {
        bmml_dma_release_stream(dma, stream);
        return tmr_status;
    }

    dac_taken[slot] = true;
    dac_t *dac = &dac_pool[slot];
    dac->res = &dac_res[idx];
    dac->wave_buffer = wave_buffer;
    dac->num_points = num_points;
    dac->dma.res = &dma_res[dma_idx];
    dac->dma.stream = dma_stream;
    dac->dma.num_stream = stream;
    dac->dma.channel = DMA_DEFAULT_CHANNEL;

    // RCC: DAC & DMA
    *dac->res->rcc_reg |= dac->res->rcc_mask;
    *dac->dma.res->rcc_reg |= dac->dma.res->rcc_mask;
    
    // disable DMA stream and reset settings
    dma_disable_stream(dac->dma.stream);
    dma_clear_stream_flags(dac->dma.res->reg, dac->dma.num_stream);

    // Setup addresses
    // (Data Holding Register, 12-bit, Right-aligned, Channel 1
    dac->dma.stream->PAR = (uint32_t)&dac->res->reg->DHR12R1;
    dac->dma.stream->M0AR = (uint32_t)dac->wave_buffer;         // wave of signal
    dac->dma.stream->NDTR = dac->num_points;                    // num of points

    // DMA config
    uint32_t cr = 0;
    cr |= (dac->dma.channel << DMA_SxCR_CHSEL_Pos);             // select channel (7)
    cr |= DMA_SxCR_MINC;                                        // increment memory
    cr &= ~DMA_SxCR_PINC;                                       // disable peref increment
    cr |= (1U << DMA_SxCR_DIR_Pos);                             // direction M2P 
    cr |= DMA_SxCR_CIRC;                                        // mode Circular
    cr |= (1U << DMA_SxCR_MSIZE_Pos);                           // size memory 16 bit (half-word)
    cr |= (1U << DMA_SxCR_PSIZE_Pos);                           // size periph 16 bit (half-word)
    dac->dma.stream->CR = cr;
    
    // Configure timer
    dac->timer = timer;
    dac->timer->res->reg->CR2 &= ~TIM_CR2_MMS;
    dac->timer->res->reg->CR2 |= (2U << TIM_CR2_MMS_Pos); // 010 : update event is used as trigger output
    // DAC config
    dac->res->reg->CR &= ~0xFFFFU;  // TODO: add 2 channel
    dac->res->reg->CR |= DAC_CR_TEN1;   // enable trigger TEN1 = 1
    // Enable DMA in DAC
    dac->res->reg->CR |= DAC_CR_DMAEN1;
    // Enable DAC module
    dac->res->reg->CR |= DAC_CR_EN1;
    // Enable timer
    timer_basic_start(dac->timer);

    if(out) *out = dac;

    return BMML_OK;
}

bmml_status_t dac_dma_release(dac_t *dac) {
    if(dac == NULL || dac->res == NULL) return BMML_INVALID_ARG;

    int slot = dac_slot(dac->res->reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(!dac_taken[slot]) return BMML_INVALID_ARG;

    dac->res->reg->CR &= ~(DAC_CR_EN1 | DAC_CR_DMAEN1 | DAC_CR_TEN1);

    if(dma_disable_stream(dac->dma.stream) != BMML_OK) return BMML_TIMEOUT;

    dma_clear_stream_flags(dac->dma.res->reg, dac->dma.num_stream);
    bmml_dma_release_stream(dac->dma.res->reg, dac->dma.num_stream);

    timer_basic_stop(dac->timer);
    if((timer_basic_release(dac->timer)) != BMML_OK) return BMML_INVALID_ARG;
    dac->timer->res->reg->CR2 &= ~TIM_CR2_MMS;
    dac->timer = NULL;
    dac_taken[slot] = false;
    dac_pool[slot] = (dac_t){0};

    return BMML_OK;
}