#include "bmml_dma_shared.h"
#include "bmml_status.h"
#include "stm32f446xx.h"
#include <stddef.h>

static bool dma1_stream_taken[DMA_STREAM_COUNT];
static bool dma2_stream_taken[DMA_STREAM_COUNT];

static DMA_Stream_TypeDef* const dma1_streams[] = {
    DMA1_Stream0,DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, 
    DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7
};

static DMA_Stream_TypeDef* const dma2_streams[] = {
    DMA2_Stream0,DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, 
    DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
};

static const IRQn_Type dma1_irqn[DMA_STREAM_COUNT] = {
    DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
    DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn
};
static const IRQn_Type dma2_irqn[DMA_STREAM_COUNT] = {
    DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
    DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
};

typedef struct {
    dma_stream_cb_t cb;
    void *ctx;
} dma_owner_t;

static dma_owner_t dma1_owner[DMA_STREAM_COUNT];
static dma_owner_t dma2_owner[DMA_STREAM_COUNT];

bmml_status_t bmml_dma_acquire_stream(DMA_TypeDef *dma, uint8_t stream, dma_stream_cb_t cb, void *ctx, DMA_Stream_TypeDef **out_stream) {
    if(out_stream) *out_stream = NULL;
    if(stream >= DMA_STREAM_COUNT) return BMML_INVALID_ARG;

    dma_owner_t *owner_table;
    bool *taken_table;
    DMA_Stream_TypeDef* const *streams;

    if(dma == DMA1) { owner_table = dma1_owner; taken_table = dma1_stream_taken; streams = dma1_streams; }
    else if(dma == DMA2) { owner_table = dma2_owner; taken_table = dma2_stream_taken; streams = dma2_streams; }
    else return BMML_INVALID_ARG;

    if(taken_table[stream]) return BMML_BUSY;
    taken_table[stream] = true;
    owner_table[stream] = (dma_owner_t){ .cb = cb, .ctx = ctx };
    NVIC_EnableIRQ((dma == DMA1) ? dma1_irqn[stream] : dma2_irqn[stream]);
    if(out_stream) *out_stream = streams[stream];

    return BMML_OK;
}

bmml_status_t bmml_dma_release_stream(DMA_TypeDef *dma, uint8_t stream) {
    if(stream >= DMA_STREAM_COUNT) return BMML_INVALID_ARG;

    dma_owner_t *owner_table;
    bool *taken_table;
    if(dma == DMA1) { owner_table = dma1_owner; taken_table = dma1_stream_taken; }
    else if(dma == DMA2) { owner_table = dma2_owner; taken_table = dma2_stream_taken; }
    else return BMML_INVALID_ARG;

    if(!taken_table[stream]) return BMML_INVALID_ARG;
    taken_table[stream] = false;
    NVIC_DisableIRQ((dma == DMA1) ? dma1_irqn[stream] : dma2_irqn[stream]);
    owner_table[stream] = (dma_owner_t){0};

    return BMML_OK;
}

static inline void dma_dispatch(dma_owner_t *owner_table, uint8_t stream) {
    dma_stream_cb_t cb = owner_table[stream].cb;
    if(cb != NULL) cb(owner_table[stream].ctx);
}

void DMA1_Stream0_IRQHandler(void) { dma_dispatch(dma1_owner, 0); }
void DMA1_Stream1_IRQHandler(void) { dma_dispatch(dma1_owner, 1); }
void DMA1_Stream2_IRQHandler(void) { dma_dispatch(dma1_owner, 2); }
void DMA1_Stream3_IRQHandler(void) { dma_dispatch(dma1_owner, 3); }
void DMA1_Stream4_IRQHandler(void) { dma_dispatch(dma1_owner, 4); }
void DMA1_Stream5_IRQHandler(void) { dma_dispatch(dma1_owner, 5); }
void DMA1_Stream6_IRQHandler(void) { dma_dispatch(dma1_owner, 6); }
void DMA1_Stream7_IRQHandler(void) { dma_dispatch(dma1_owner, 7); }

void DMA2_Stream0_IRQHandler(void) { dma_dispatch(dma2_owner, 0); }
void DMA2_Stream1_IRQHandler(void) { dma_dispatch(dma2_owner, 1); }
void DMA2_Stream2_IRQHandler(void) { dma_dispatch(dma2_owner, 2); }
void DMA2_Stream3_IRQHandler(void) { dma_dispatch(dma2_owner, 3); }
void DMA2_Stream4_IRQHandler(void) { dma_dispatch(dma2_owner, 4); }
void DMA2_Stream5_IRQHandler(void) { dma_dispatch(dma2_owner, 5); }
void DMA2_Stream6_IRQHandler(void) { dma_dispatch(dma2_owner, 6); }
void DMA2_Stream7_IRQHandler(void) { dma_dispatch(dma2_owner, 7); }