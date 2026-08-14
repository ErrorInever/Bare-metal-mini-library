#ifndef BMML_I2C_H
#define BMML_I2C_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"

#define I2C_RES_COUNT 3

typedef struct {
    I2C_TypeDef *reg;
    volatile uint32_t *rcc_reg;
    uint32_t rcc_mask;
    IRQn_Type irqn_ev;
    IRQn_Type irqn_er;
    uint8_t  dma_tx_stream;   // Stream + channel for TX
    uint32_t dma_tx_channel;
    uint8_t  dma_rx_stream;   // RX 
    uint32_t dma_rx_channel;
} i2c_res_t;

static const i2c_res_t i2c_res[] = {
    {I2C1, &RCC->APB1ENR, RCC_APB1ENR_I2C1EN, I2C1_EV_IRQn, I2C1_ER_IRQn, 6, 1, 0, 1},
    {I2C2, &RCC->APB1ENR, RCC_APB1ENR_I2C2EN, I2C2_EV_IRQn, I2C2_ER_IRQn, 7, 7, 2, 7},
    {I2C3, &RCC->APB1ENR, RCC_APB1ENR_I2C3EN, I2C3_EV_IRQn, I2C3_ER_IRQn, 4, 3, 2, 3},
};

// Get index of I2C res
static inline int i2c_res_idx(const I2C_TypeDef *reg) {
    for(int i = 0; i < (int)I2C_RES_COUNT; i++) {
        if(i2c_res[i].reg == reg) return i;
    }
    return -1;
}


#ifdef __cplusplus
}
#endif


#endif /* BMML_I2C_H */