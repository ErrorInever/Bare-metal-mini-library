#include "bmml_utilities.h"
#include "stm32f446xx.h"
#include <stdint.h>



uint32_t get_apb1_clock_hz(void) {
    static const uint16_t ahb_div[] = {
        1, 1, 1, 1, 1, 1, 1, 1,
        2, 4, 8, 16, 64, 128, 256, 512
    };

    static const uint8_t apb_div[] = {
        1, 1, 1, 1,
        2, 4, 8, 16
    };

    uint32_t cfgr = RCC->CFGR;

    uint32_t hpre  = (cfgr >> RCC_CFGR_HPRE_Pos) & 0xF;
    uint32_t ppre1 = (cfgr >> RCC_CFGR_PPRE1_Pos) & 0x7;

    uint32_t hclk = SystemCoreClock / ahb_div[hpre];

    return hclk / apb_div[ppre1];
}

uint32_t get_apb2_clock_hz(void) {
    static const uint16_t ahb_div[] = {
        1, 1, 1, 1, 1, 1, 1, 1,
        2, 4, 8, 16, 64, 128, 256, 512
    };

    static const uint8_t apb_div[] = {
        1, 1, 1, 1,
        2, 4, 8, 16
    };

    uint32_t cfgr = RCC->CFGR;

    uint32_t hpre  = (cfgr >> RCC_CFGR_HPRE_Pos) & 0xF;
    uint32_t ppre2 = (cfgr >> RCC_CFGR_PPRE2_Pos) & 0x7;

    uint32_t hclk = SystemCoreClock / ahb_div[hpre];

    return hclk / apb_div[ppre2];
}