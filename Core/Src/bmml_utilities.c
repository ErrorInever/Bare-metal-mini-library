#include "bmml_utilities.h"
#include "stm32f446xx.h"
#include <stdint.h>

#define ADC_CLOCK_MAX_HZ 36000000U


static const uint16_t ahb_div[] = { 1,1,1,1,1,1,1,1, 2,4,8,16,64,128,256,512 };
static const uint8_t  apb_div[] = { 1,1,1,1, 2,4,8,16 };


static uint32_t get_hclk_hz(void) {
    uint32_t hpre = (RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0xF;
    return SystemCoreClock / ahb_div[hpre];
}

uint32_t get_apb1_clock_hz(void) {
    uint32_t ppre1 = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7;
    return get_hclk_hz() / apb_div[ppre1];
}

uint32_t get_apb2_clock_hz(void) {
    uint32_t ppre2 = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x7;
    return get_hclk_hz() / apb_div[ppre2];
}

uint32_t bmml_calc_adc_prescaler(uint32_t apb2_hz) {
    static const uint8_t div_values[4] = {2, 4, 6, 8};   // ADCPRE = 0,1,2,3

    for (uint8_t i = 0; i < 4; i++) {
        if (apb2_hz / div_values[i] <= ADC_CLOCK_MAX_HZ) {
            return i;   // return code ADCPRE
        }
    }
    return 3;   // div 8 (minimal)
}