#ifndef BMML_IRQ_SHARED_H
#define BMML_IRQ_SHARED_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef void (*shared_irq_handler_t)(void);

void bmml_register_tim6_dac_handler(shared_irq_handler_t timer_part, shared_irq_handler_t dac_part);
void bmml_unregister_tim6_dac_handler(bool clear_timer, bool clear_dac);

#ifdef __cplusplus
}
#endif


#endif /* BMML_IRQ_SHARED_H */