#ifndef BMML_IRQ_SHARED_H
#define BMML_IRQ_SHARED_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* One NVIC priority for every interrupt the library enables (peripherals, DMA streams, EXTI).
   Equal priority means these ISRs never preempt each other, so the read-modify-write of
   CR1/CR2/... shared between a peripheral ISR and its DMA ISR cannot be torn.
   Do not give an application ISR a numerically lower value if it touches BMML peripherals. */
#ifndef BMML_IRQ_PRIORITY
#define BMML_IRQ_PRIORITY 5U
#endif

typedef void (*shared_irq_handler_t)(void);

void bmml_register_tim6_dac_handler(shared_irq_handler_t timer_part, shared_irq_handler_t dac_part);
void bmml_unregister_tim6_dac_handler(bool clear_timer, bool clear_dac);

#ifdef __cplusplus
}
#endif


#endif /* BMML_IRQ_SHARED_H */