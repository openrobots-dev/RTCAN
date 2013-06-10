#ifndef _RTCAN_LLD_TIM_H_
#define _RTCAN_LLD_TIM_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   RTCAND1 timer driver selection.
 * @note    The default is TIM3.
 */
#if !defined(RTCAN_STM32_RTCAND1_TIM) || defined(__DOXYGEN__)
#define RTCAN_STM32_RTCAND1_TIM             TIM3
#endif

/**
 * @brief   RTCAND1 timer interrupt priority level setting.
 * @note    The default is 3.
 */
#if !defined(RTCAN_STM32_RTCAND1_TIM_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define RTCAN_STM32_RTCAND1_TIM_IRQ_PRIORITY         4
#endif

/**
 * @brief   RTCAND2 timer driver selection.
 * @note    The default is TIM4.
 */
#if !defined(RTCAN_STM32_RTCAND2_TIM) || defined(__DOXYGEN__)
#define RTCAN_STM32_RTCAND2_TIM             TIM4
#endif

/**
 * @brief   RTCAND2 timer interrupt priority level setting.
 */
#if !defined(RTCAN_STM32_RTCAND2_TIM_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define RTCAN_STM32_RTCAND2_TIM_IRQ_PRIORITY         4
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must have been activated using @p gptStart().
 * @pre     The GPT unit must have been running in continuous mode using
 *          @p gptStartContinuous().
 * @post    The GPT unit interval is changed to the new value.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 * @notapi
 */
#define gpt_lld_change_interval(gptp, interval)                               \
  ((gptp)->tim->ARR = (uint16_t)((interval) - 1))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void rtcan_lld_tim_init(void);
  void rtcan_lld_tim_start(RTCANDriver * rtcanp);
  void rtcan_lld_tim_stop(RTCANDriver * rtcanp);
  void rtcan_lld_tim_start_timer(RTCANDriver * rtcanp);
  void rtcan_lld_tim_stop_timer(RTCANDriver * rtcanp);
  void rtcan_lld_tim_set_period(RTCANDriver * rtcanp, rtcan_cnt_t interval);
  void rtcan_lld_tim_set_counter(RTCANDriver * rtcanp, rtcan_cnt_t cnt);
#ifdef __cplusplus
}
#endif

#endif /* _RTCAN_LLD_TIM_H_ */

/** @} */
