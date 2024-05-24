/*
 * stm32f103x6_GPTim2_4.h
 *
 *  Created on: Mar 1, 2024
 *      Author: Hossam Eid
 */

#ifndef MCAL_GP_TIMERS_2_TO_4_STM32F103X6_GPTIM2_4_H_
#define MCAL_GP_TIMERS_2_TO_4_STM32F103X6_GPTIM2_4_H_

/** @defgroup INCLUDES
  * @{
  */
#include "stm32f103x6.h"
#include "NVIC/stm32f103x6_NVIC.h"
/**
  * @}
  */


/**
======================================================================================================================
* @Func_name	  : MCAL_GPTIM_Init
* @brief		    : Function to initialize the GPTIM registers.
* @param [in]	  : TIMx: Timer instance to be initialized
======================================================================================================================
*/
void MCAL_GPTIM_Init(volatile GPTim_Typedef* TIMx, void (* callback)());

/**
======================================================================================================================
* @Func_name	  : MCAL_GPTIM_Start
* @brief		    : Function to start the timer counting
* @param [in]	  : TIMx: Timer instance to be started
======================================================================================================================
*/
void MCAL_GPTIM_Start(volatile GPTim_Typedef* TIMx);
#endif /* MCAL_GP_TIMERS_2_TO_4_STM32F103X6_GPTIM2_4_H_ */
