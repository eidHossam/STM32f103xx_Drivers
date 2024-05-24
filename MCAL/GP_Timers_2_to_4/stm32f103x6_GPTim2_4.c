/*
 * stm32f103x6_GPTim2_4.c
 *
 *  Created on: Mar 1, 2024
 *      Author: Hossam Eid
 */


/** @defgroup INCLUDES
  * @{
  */
#include "stm32f103x6_GPTim2_4.h"

/**
  * @}
  */

/** @defgroup LOCAL_MACROS
  * @{
  */
#define TIMX_CEN_POS    0       /*Enable counter*/

#define TIMX_OPM_MASK (1 << 3)  /*One Pulse Mode*/
#define TIMX_URS_MASK (1 << 2)  /*Only counter overflow generates an update interrupt if enabled.*/
#define TIMX_UIE_MASk (1 << 0)  /*Update interrupt enable*/
#define TIMX_UIF_MASK (1 << 0)  /*Update interrupt flag*/
/**
  * @}
  */


/** @defgroup GLOBAL_VARiABLES
  * @{
  */
void (* GPtr_TIM2_Callback)();
void (* GPtr_TIM3_Callback)();
void (* GPtr_TIM4_Callback)();

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
void MCAL_GPTIM_Init(volatile GPTim_Typedef* TIMx, void (* callback)())
{
    uint16 tempReg;
    /*Start the instance clock*/
    if(TIMx == TIM2)
    {
        APB1_PERI_CLOCK_EN(APB1_TIM2);
        GPtr_TIM2_Callback = callback;
        MCAL_NVIC_EnableIRQ(NVIC_TIM2_IVT_INDEX);

    }else if(TIMx == TIM3)
    {
        APB1_PERI_CLOCK_EN(APB1_TIM3);
        GPtr_TIM3_Callback = callback;
        MCAL_NVIC_EnableIRQ(NVIC_TIM3_IVT_INDEX);

    }else if(TIMx == TIM4)
    {
        APB1_PERI_CLOCK_EN(APB1_TIM4);
        GPtr_TIM4_Callback = callback;
        MCAL_NVIC_EnableIRQ(NVIC_TIM4_IVT_INDEX);
    }else{

    }

    tempReg = 0;
    tempReg |= TIMX_OPM_MASK | TIMX_URS_MASK;
    
    /*One pulse mode with only counter overflow to generate an update event*/
    TIMx->CR1 = tempReg;

    /*Enable update event interrupt*/
    tempReg = 0;
    tempReg |= TIMX_UIE_MASk;
    TIMx->DIER = tempReg;

    /*This timing is only valid for 8MHZ Clock*/
    TIMx->PSC = 4000;
    TIMx->ARR = 20000;
}

/**
======================================================================================================================
* @Func_name	  : MCAL_GPTIM_Start
* @brief		    : Function to start the timer counting
* @param [in]	  : TIMx: Timer instance to be started
======================================================================================================================
*/
void MCAL_GPTIM_Start(volatile GPTim_Typedef* TIMx)
{
    SET_BIT(TIMx->CR1, TIMX_CEN_POS);
}


/** @defgroup TIM IRQ HANDLER
  * @{
  */

void TIM2_IRQHandler(void)
{
    GPtr_TIM2_Callback();
}

void TIM3_IRQHandler(void)
{
    GPtr_TIM3_Callback();

}

void TIM4_IRQHandler(void)
{
    GPtr_TIM4_Callback();

}

/**
  * @}
  */
