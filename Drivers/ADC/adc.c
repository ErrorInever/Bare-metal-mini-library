#include "adc.h"
#include "bmml_res_adc.h"
#include "bmml_status.h"
#include <stddef.h>
#include <stdint.h>
#include "bmml_dma_shared.h"
#include "bmml_utilities.h"
#include "stm32f446xx.h"

#define ADC_COUNT 3

static void adc_dma_isr(void *ctx);

static adc_t adc_pool[ADC_COUNT];
static bool adc_taken[ADC_COUNT];

static int adc_slot(const ADC_TypeDef *reg) {
    if(reg == ADC1) return 0;
    if(reg == ADC2) return 1;
    if(reg == ADC3) return 2;
    return -1;
}

static bmml_status_t adc_configure_queue_simple(adc_t *adc, uint8_t global_sampling_time) {
    // If channels > 1 enable Scan MODE
    if(adc->num_channels > 1) adc->res->reg->CR1 |= ADC_CR1_SCAN;

    // Setup length queue in SQR1 
    adc->res->reg->SQR1 &= ~ADC_SQR1_L;
    adc->res->reg->SQR1 |= ((adc->num_channels - 1) << ADC_SQR1_L_Pos);

    // Reset channels
    adc->res->reg->SQR3 = 0;

    for(uint8_t i = 0; i < adc->num_channels; i++) {
        uint8_t channel_num = adc->adc_channels[i].channel_number;
        adc->res->reg->SQR3 |= (channel_num << (i * 5));
        // Setup sample time
        if(channel_num <= 9) {
            adc->res->reg->SMPR2 = (adc->res->reg->SMPR2 & ~(7U << (channel_num * 3))) | (global_sampling_time << (channel_num * 3));
        }else if (channel_num <= 18) {
            uint8_t pos = (channel_num - 10) * 3;
            adc->res->reg->SMPR1 = (adc->res->reg->SMPR1 & ~(7U << pos)) | (global_sampling_time << pos);
        }else {
            return BMML_INVALID_ARG;
        }
    }
    return BMML_OK;
}


bmml_status_t adc_dma_acquire(ADC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream, adc_mode_t mode, adc_channel_config_t *channels, 
    uint8_t num_channels, uint16_t *data_buffer, uint8_t sample_time, adc_callback_t cb, adc_t **out) {

    if(out) *out = NULL;
    if(reg == NULL || dma == NULL) return BMML_INVALID_ARG;
    if (channels == NULL || num_channels == 0 || num_channels > 6) return BMML_INVALID_ARG;

    if (mode == TIME_TRIGGER) return BMML_INVALID_ARG;   // TODO: not implemented — reject before touching any resource
    if (mode != CONTINUOUS && mode != SCAN) return BMML_INVALID_ARG; 

    int slot = adc_slot(reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(adc_taken[slot]) return BMML_BUSY;

    int idx = adc_res_idx(reg);
    if(idx < 0) return BMML_INVALID_ARG;

    if (dma != DMA2 || stream != adc_res[idx].dma_stream) return BMML_INVALID_ARG;

    int dma_idx = dma_res_idx(dma);
    if(dma_idx < 0) return BMML_INVALID_ARG;

    // Acquire DMA stream
    DMA_Stream_TypeDef *dma_stream;
    bmml_status_t st = bmml_dma_acquire_stream(dma, stream, adc_dma_isr, NULL, &dma_stream);
    if(st != BMML_OK) return st;

    // Init
    adc_taken[slot] = true;
    adc_t *adc = &adc_pool[slot];
    adc->res = &adc_res[idx];
    adc->dma.res = &dma_res[dma_idx];
    adc->dma.stream = dma_stream;
    adc->dma.num_stream = stream;
    adc->mode = mode;
    adc->adc_channels = channels;
    adc->num_channels = num_channels;
    adc->data_buffer = data_buffer;
    adc->dma.channel = adc->res->dma_channel;
    adc->callback = cb;

    // Enable RCC: ADC & DMA
    *adc->res->rcc_reg |= adc->res->rcc_mask;
    *adc->dma.res->rcc_reg |= adc->dma.res->rcc_mask;

    // Disable DMA stream and reset settings
    dma_disable_stream(adc->dma.stream);
    dma_clear_stream_flags(adc->dma.res->reg, adc->dma.num_stream);

    // DMA config
    uint32_t cr = 0;
    cr |= (adc->dma.channel << DMA_SxCR_CHSEL_Pos);             // select channel (7)
    cr |= DMA_SxCR_MINC;                                        // increment memory
    cr &= ~DMA_SxCR_PINC;                                       // disable peref increment
    cr |= 0U;                                                   // direction P2M 
    cr |= DMA_SxCR_CIRC;                                        // mode Circular
    cr |= DMA_SxCR_TCIE;                                        // enable interrupt Transmit complete
    cr |= DMA_SxCR_TEIE;                                        // enable interrupt Transport error
    cr |= (1U << DMA_SxCR_MSIZE_Pos);                           // size memory 16 bit (half-word)
    cr |= (1U << DMA_SxCR_PSIZE_Pos);                           // size periph 16 bit (half-word)
    adc->dma.stream->CR = cr;

    // ADC configure
    // Setup freq (max 18Mhz)
    ADC->CCR &= ~ADC_CCR_ADCPRE; // reset prescaler
    uint32_t adcpre = bmml_calc_adc_prescaler(get_apb2_clock_hz());
    ADC->CCR |= (adcpre << ADC_CCR_ADCPRE_Pos);
    // Setup resolution
    adc->res->reg->CR1 &= ~ADC_CR1_RES; // max resolution 12 bit (00)

    // Setup Queue
    bmml_status_t queue_status = adc_configure_queue_simple(adc, sample_time);
    if (queue_status != BMML_OK) {
        bmml_dma_release_stream(dma, stream);
        adc_taken[slot] = false;
        adc_pool[slot] = (adc_t){0};
        return queue_status;
    }

    if(mode == CONTINUOUS) {
        adc->res->reg->CR2 |= ADC_CR2_CONT;
    }else if (mode == TIME_TRIGGER) {
        // TODO: TIM (EXTEN/EXTSEL)
        return BMML_INVALID_ARG;
    }

    // Allow DMA request generation for ADC
    // Allow continuous generation of requests (DDS = DMA Disable Selection)
    adc->res->reg->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    // Enable ADC
    adc->res->reg->CR2 |= ADC_CR2_ADON;
    // Enable DMA
    adc->dma.stream->CR |= DMA_SxCR_EN;

    return BMML_OK;
}

bmml_status_t adc_dma_release(adc_t *adc) {
    if(adc == NULL || adc->res == NULL) return BMML_INVALID_ARG;

    int slot = adc_slot(adc->res->reg);
    if(slot < 0) return BMML_INVALID_ARG;
    if(!adc_taken[slot]) return BMML_INVALID_ARG;

    adc->res->reg->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON);

    if(dma_disable_stream(adc->dma.stream) != BMML_OK) return BMML_TIMEOUT;

    dma_clear_stream_flags(adc->dma.res->reg, adc->dma.num_stream);
    bmml_dma_release_stream(adc->dma.res->reg, adc->dma.num_stream);

    adc_taken[slot] = false;
    adc_pool[slot] = (adc_t){0};

    return BMML_OK;
}

static void adc_dma_isr(void *ctx) {
    adc_t *adc = (adc_t *)ctx;

    volatile uint32_t *isr  = (adc->dma.num_stream <= 3) ? &adc->dma.res->reg->LISR  : &adc->dma.res->reg->HISR;
    volatile uint32_t *ifcr = (adc->dma.num_stream <= 3) ? &adc->dma.res->reg->LIFCR : &adc->dma.res->reg->HIFCR;
    uint32_t teif = adc->res->tcif_mask >> 2;

    if(*isr & teif) {
        *ifcr = teif;
        if(adc->callback != NULL) adc->callback(adc);
        return;
    }
    if(*isr & adc->res->tcif_mask) {
        *ifcr = adc->res->tcif_mask;
        if(adc->callback != NULL) adc->callback(adc);
    }
}