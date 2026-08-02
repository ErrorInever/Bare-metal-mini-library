#ifndef TIMER_H
#define TIMER_H


#ifdef __cplusplus
extern "C" {
#endif

#define TIMER_BASIC_MAX_PERIOD_MS 6553U     // ARR 16-bit @ 10kHz tick

#include <stdint.h>
#include "stm32f446xx.h"
#include "bmml_res_timer.h"
#include "bmml_status.h"

/* ------------------------------------- BASIC TIMER ------------------------------------- */

// User callback definition for basic timer
typedef void (*timer_basic_callback_t)(void);

// Basic timer object
/**
 * @struct timer_basic_t
 * @brief Runtime handle for a basic timer (TIM6 or TIM7) acquired from the pool.
 *
 * Returned by timer_basic_acquire(). Must not be constructed by the caller directly —
 * the pool guarantees exclusive ownership of the underlying peripheral.
 */
typedef struct {
    const timer_res_t *res;             /**< Pointer to the static resource descriptor of the acquired instance. */
    timer_basic_callback_t callback;    /**< User callback invoked from the Update-event ISR, or NULL. */
    bool status;                        /**< Current run state: true if the counter is enabled (CEN set). */
} timer_basic_t;


/**
 * @brief Acquires and configures a basic timer (TIM6 or TIM7) with a periodic update event.
 *
 * Enables the peripheral clock, computes PSC/ARR for a fixed 10 kHz tick resolution (0.1 ms per tick),
 * arms the Update interrupt, and enables the corresponding NVIC vector. If `reg` is TIM6, this also
 * registers the timer's ISR fragment with the shared TIM6/DAC vector dispatcher (see bmml_irq_shared.h).
 *
 * The timer is left stopped (CEN = 0); call timer_basic_start() to begin counting.
 *
 * @pre `reg` must be TIM6 or TIM7 and must not already be held by another handle.
 * @param[in]  reg       Target basic timer instance (TIM6 or TIM7).
 * @param[in]  period_ms Update period in milliseconds. Valid range: 1..TIMER_BASIC_MAX_PERIOD_MS.
 * @param[in]  cb        Callback invoked from the ISR on every Update event. May be NULL.
 * @param[out] out       Receives the acquired handle on success; set to NULL on any failure.
 * @return BMML_OK on success, BMML_INVALID_ARG for a bad register/period, BMML_BUSY if already acquired.
 */
bmml_status_t timer_basic_acquire(TIM_TypeDef *reg, const uint32_t period_ms, timer_basic_callback_t cb, timer_basic_t **out);


/**
 * @brief Releases a previously acquired basic timer, stopping it and returning it to the pool.
 *
 * Disables the counter and the Update interrupt, disables the NVIC vector, and (for TIM6) unregisters
 * the shared TIM6/DAC ISR fragment. The handle is invalidated — after this call `timer` must not be reused.
 *
 * @param[in] timer Handle previously obtained from timer_basic_acquire().
 * @return BMML_OK on success, BMML_INVALID_ARG if `timer` is NULL, unrecognized, or already released.
 */
bmml_status_t timer_basic_release(timer_basic_t *timer);


/**
 * @brief Blocking, polling-based delay using the timer's free-running counter.
 *
 * Temporarily starts the counter if it was stopped, disables the Update interrupt for the
 * duration of the wait (so a concurrently configured callback is not fired mid-delay), and busy-waits
 * until `ms` milliseconds have elapsed, measured against the timer's current tick rate.
 *
 * @pre The requested delay must fit within one period of the timer's currently configured ARR
 *      (i.e. `ms * 10 <= ARR + 1`); longer delays are rejected rather than silently wrapping.
 * @param[in] timer Handle previously obtained from timer_basic_acquire().
 * @param[in] ms    Delay duration in milliseconds.
 * @return BMML_OK on success, BMML_INVALID_ARG if the delay exceeds the timer's configured period.
 */
bmml_status_t timer_basic_delay_poling(const timer_basic_t *timer, uint32_t ms);


/**
 * @brief Starts the counting mechanism for a basic timer.
 * @pre `timer` must be a valid handle from timer_basic_acquire().
 * @param[in,out] timer Handle to start; its `status` field is set to true.
 */
static inline void timer_basic_start(timer_basic_t *timer) {
    timer->res->reg->CR1 |= TIM_CR1_CEN;
    timer->status = true;
}


/**
 * @brief Stops the counting mechanism and clears pending update flags for a basic timer.
 * @pre `timer` must be a valid handle from timer_basic_acquire().
 * @param[in,out] timer Handle to stop; its `status` field is set to false.
 */
static inline void timer_basic_stop(timer_basic_t *timer) {
    timer->res->reg->CR1 &= ~TIM_CR1_CEN;
    timer->res->reg->SR &= ~TIM_SR_UIF;
    timer->status = false;
}


/**
 * @brief Dynamically changes the auto-reload period value for a basic timer.
 * * Updates the ARR register and forces a software update execution to apply shadow registers.
 * * @param[in] timer     Pointer to the basic timer configuration structure.
 * @param[in] period_ms New target period in milliseconds.
 */
static inline bmml_status_t timer_basic_set_period_ms(const timer_basic_t *timer, const uint32_t period_ms) {
    if (period_ms == 0 || period_ms > TIMER_BASIC_MAX_PERIOD_MS) return BMML_INVALID_ARG;
    timer->res->reg->ARR = (period_ms * 10U) - 1U;    // Update ARR
    timer->res->reg->EGR |= TIM_EGR_UG;             // forced update shadow registers
    timer->res->reg->SR &= ~TIM_SR_UIF;
    return BMML_OK;
}

/* --------------------------------------------------------------------------------------- */


/* ------------------------------------- PWM TIMER ------------------------------------- */


/**
 * @struct timer_pwm_t
 * @brief Runtime handle for a PWM-capable timer instance acquired from the pool.
 *
 * One handle owns the whole physical timer (shared PSC/ARR, hence shared PWM frequency),
 * not a single channel. Individual output channels (1..num_channels) are configured
 * independently via timer_pwm_channel_config() and tracked in `channel_enabled_mask`.
 */
typedef struct {
    const timer_res_t *res;             /**< Pointer to the static resource descriptor of the acquired instance. */
    uint32_t channel_enabled_mask;      /**< Bitmask of configured channels; bit (n-1) set means channel n is configured. */
    uint32_t channel_mode;              /**< Reserved for future per-channel mode tracking (currently unused). */
    bool status;                        /**< Current run state: true if the counter is enabled (CEN set). */
} timer_pwm_t;


/**
 * @brief Acquires a PWM-capable timer instance and configures it for a target output frequency.
 *
 * Enables the peripheral clock and searches for the PSC/ARR pair that achieves `freq_hz` while
 * maximizing duty-cycle resolution (i.e. maximizing ARR within the register's bit width). Basic
 * timers (TIM6/TIM7) are rejected — they have no CCR/CCMR registers. The timer is left stopped and
 * with no channels configured; call timer_pwm_channel_config() per channel and timer_pwm_start() to run.
 *
 * @pre `reg` must be a GP or Advanced category timer and must not already be held by another handle.
 * @param[in]  reg     Target timer instance (TIM1-5, TIM8-14).
 * @param[in]  freq_hz Desired PWM output frequency in Hz. Must be reachable given the timer's clock
 *                     (freq_hz <= timer clock) and a PSC in range 0..0xFFFF that yields a non-zero ARR.
 * @param[out] out     Receives the acquired handle on success; set to NULL on any failure.
 * @return BMML_OK on success, BMML_INVALID_ARG for a bad/basic-category register or unreachable frequency,
 *         BMML_BUSY if already acquired.
 */
bmml_status_t timer_pwm_acquire(TIM_TypeDef *reg, uint32_t freq_hz, timer_pwm_t **out);

/**
 * @brief Releases a previously acquired PWM timer, disabling the counter and all configured channels.
 *
 * Stops the counter, disables CCER for every channel currently marked in `channel_enabled_mask`,
 * and (for TIM1/TIM8) clears the Main Output Enable bit (BDTR.MOE). The handle is invalidated —
 * after this call `timer` must not be reused.
 *
 * @param[in] timer Handle previously obtained from timer_pwm_acquire().
 * @return BMML_OK on success, BMML_INVALID_ARG if `timer` is NULL, unrecognized, or already released.
 */
bmml_status_t timer_pwm_release(timer_pwm_t *timer);


/**
 * @brief Configures one output-compare channel for PWM generation (PWM Mode 1) with preload enabled.
 *
 * Sets the initial duty cycle, configures OCxM/OCxPE in CCMR1/CCMR2, and marks the channel as
 * configured in `channel_enabled_mask`. Does NOT enable the physical output — call
 * timer_pwm_channel_enable() afterward to drive the pin. For TIM1/TIM8, also sets BDTR.MOE.
 *
 * @note The PWM frequency is fixed at acquire time via timer_pwm_acquire() and shared by all
 *       channels of this timer instance; this function does not alter PSC/ARR.
 * @param[in,out] timer   Handle previously obtained from timer_pwm_acquire().
 * @param[in]     channel Channel number, 1-based (1..res->num_channels).
 * @param[in]     duty    Initial duty cycle as a percentage (0..100); values above 100 are clamped.
 * @return BMML_OK on success, BMML_INVALID_ARG for a NULL handle or out-of-range channel,
 *         BMML_BUSY if the channel is already configured.
 */
bmml_status_t timer_pwm_channel_config(timer_pwm_t *timer, uint8_t channel, uint32_t duty);


/**
 * @brief Enables the physical output for a previously configured PWM channel (sets CCER.CCxE).
 * @param[in] timer   Handle previously obtained from timer_pwm_acquire().
 * @param[in] channel Channel number, 1-based (1..4).
 * @return BMML_OK on success, BMML_INVALID_ARG for a NULL handle or out-of-range channel,
 *         BMML_ERROR if the channel was never configured via timer_pwm_channel_config().
 */
bmml_status_t timer_pwm_channel_enable(const timer_pwm_t *timer, uint8_t channel);


/**
 * @brief Disables the physical output for a configured PWM channel (clears CCER.CCxE).
 * @param[in] timer   Handle previously obtained from timer_pwm_acquire().
 * @param[in] channel Channel number, 1-based (1..4).
 * @return BMML_OK on success, BMML_INVALID_ARG for a NULL handle or out-of-range channel,
 *         BMML_ERROR if the channel was never configured via timer_pwm_channel_config().
 */
bmml_status_t timer_pwm_channel_disable(const timer_pwm_t *timer, uint8_t channel);


/**
 * @brief Updates the duty cycle of an already-configured PWM channel.
 *
 * Recomputes CCRx from the timer's current ARR, so it stays correct even if a future API
 * changes the timer's frequency after acquire.
 *
 * @param[in] timer   Handle previously obtained from timer_pwm_acquire().
 * @param[in] channel Channel number, 1-based (1..4).
 * @param[in] duty    New duty cycle as a percentage (0..100); values above 100 are clamped.
 * @return BMML_OK on success, BMML_INVALID_ARG for a NULL handle or out-of-range channel,
 *         BMML_ERROR if the channel was never configured via timer_pwm_channel_config().
 */
bmml_status_t timer_pwm_set_duty(const timer_pwm_t *timer, uint8_t channel, uint32_t duty);


/**
 * @brief Starts the counter of a PWM timer, driving all enabled channels.
 * @pre `timer` must be a valid handle from timer_pwm_acquire().
 * @param[in,out] timer Handle to start; its `status` field is set to true.
 */
static inline void timer_pwm_start(timer_pwm_t *timer) {
    timer->res->reg->CR1 |= TIM_CR1_CEN;
    timer->status = true;
}


/**
 * @brief Stops the counter of a PWM timer and clears any pending update flag.
 * @note This stops the whole timer (all channels); it does not disable individual outputs —
 *       use timer_pwm_channel_disable() if only a single channel should stop driving its pin.
 * @pre `timer` must be a valid handle from timer_pwm_acquire().
 * @param[in,out] timer Handle to stop; its `status` field is set to false.
 */
static inline void timer_pwm_stop(timer_pwm_t *timer) {
    timer->res->reg->CR1 &= ~TIM_CR1_CEN;
    timer->res->reg->SR &= ~TIM_SR_UIF;
    timer->status = false;
}

/* --------------------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif


#endif /* TIMER_H */
