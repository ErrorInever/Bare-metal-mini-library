#ifndef ADC_H
#define ADC_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f446xx.h"
#include "bmml_status.h"
#include "bmml_dma_shared.h"
#include "bmml_res_adc.h"

/* Forward declaration of the main ADC handle structure */
struct adc_t;
typedef struct adc_t adc_t;


/**
 * @brief User-defined callback function pointer type for DMA transfer complete events.
 * @param[in] adc Pointer to the active ADC handle structure invoking the callback.
 */
typedef void (*adc_callback_t)(adc_t *adc);


/**
 * @enum adc_mode_t
 * @brief Defines the operational modes of the ADC peripheral.
 */
typedef enum {
    CONTINUOUS,   /**< Continuously converts configured channels in a loop indefinitely. */
    SCAN,         /**< Scans a sequence of multiple channels once per trigger. */
    TIME_TRIGGER  /**< Hardware timer events trigger the conversion sequence. */
} adc_mode_t;

/**
 * @struct adc_channel_config_t
 * @brief Individual hardware ADC channel configuration parameter mapping.
 */
typedef struct {
    uint8_t channel_number;                     /**< Hardware channel index (e.g., 0 for PA0, 1 for PA1). */
    uint8_t sampling_time;                      /**< Sample time selection bit pattern (defines capacitor charge cycles). */
} adc_channel_config_t;


typedef struct adc_t {
    const adc_res_t *res;
    dma_t dma;
    adc_mode_t mode;
    adc_channel_config_t *adc_channels;         /**< Pointer to user-allocated array containing channel-specific settings. */
    uint8_t num_channels;                       /**< Number of active channels defined in the array (Maximum: 6). */
    adc_callback_t callback;
    uint16_t *data_buffer;      /**< Pointer to destination memory buffer (RAM) for automated DMA streaming. */
} adc_t;



bmml_status_t adc_dma_acquire(ADC_TypeDef *reg, DMA_TypeDef *dma, uint8_t stream, adc_mode_t mode, adc_channel_config_t *channels, 
    uint8_t num_channels, uint16_t *data_buffer, uint8_t sample_time, adc_callback_t cb, adc_t **out);

static inline void adc_start(adc_t *adc) {
    if(adc->mode == CONTINUOUS || adc->mode == SCAN)
        adc->res->reg->CR2 |= ADC_CR2_SWSTART;
    // TODO: TIME_TRIGGER
}

// TODO: add release


#ifdef __cplusplus
}
#endif

#endif /*ADC_H*/