#ifndef BMML_UTILITIES_H
#define BMML_UTILITIES_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


uint32_t get_apb1_clock_hz(void);
uint32_t get_apb2_clock_hz(void);
uint32_t bmml_calc_adc_prescaler(uint32_t apb2_hz);

#ifdef __cplusplus
}
#endif


#endif /* BMML_UTILITIES_H */