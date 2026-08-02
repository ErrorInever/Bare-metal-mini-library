#include "bmml_irq_shared.h"
#include <stdio.h>

static shared_irq_handler_t tim6_dac_timer_part;
static shared_irq_handler_t tim6_dac_dac_part;

void bmml_register_tim6_dac_handler(shared_irq_handler_t timer_part, shared_irq_handler_t dac_part) {
    if(timer_part) tim6_dac_timer_part = timer_part;
    if(dac_part) tim6_dac_dac_part = dac_part;
}

void bmml_unregister_tim6_dac_handler(bool clear_timer, bool clear_dac) {
    if(clear_timer) tim6_dac_timer_part = NULL;
    if(clear_dac) tim6_dac_dac_part = NULL;
}

// TIM6 and DAC IRQ Handler
void TIM6_DAC_IRQHandler(void) {
    if(tim6_dac_timer_part) tim6_dac_timer_part();
    if(tim6_dac_dac_part) tim6_dac_dac_part();
}