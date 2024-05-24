/*
**************************************************************************************************************************
* file		: stm32f103x6_USART.c
* brief     : This files contains functions to configure the USART communication module.
* Author    : Hossam Eid
* Created on: Sep 19, 2023
**************************************************************************************************************************
* note: This module is intended for use with the STM32F103xx microcontroller series, but may be adapted for use with
* other compatible processors.
**************************************************************************************************************************
*/

/*
*===============================================
*                   Includes
*===============================================
*/
#include "stm32f103x6_USART.h"

/**************************************************************************************************************************
===============================================
*  				Global variables
*===============================================
*/
static void (* p_IRQ_callback_USART1[11])(void);
static void (* p_IRQ_callback_USART2[11])(void);
static void (* p_IRQ_callback_USART3[11])(void);

/**************************************************************************************************************************
===============================================
*  				Local functions
*===============================================
*/
static void MCAL_UART_GPIO_Set_Pins(USART_Typedef * USART, uint32 FlowCTRL)
{
    /** @defgroup GPIO configurations for USART
    * @{
    *    USARTx_TX   :  (Full duplex, Alternate function push-pull)
    *    USARTx_RX   :  (Full duplex, Input floating / Input pull-up)
    *    USARTx_RTS  :  Alternate function push-pull
    *    USARTx_CTS  :  Input floating/ Input pull-up
    * @}
    */
    GPIO_Pin_Config_t pinConfig;

    if(USART == USART1)
    {
        /** @defgroup USART1 Pins configuration
        * @{
        *   USART1_TX   : PA9
        *   USART1_RX   : PA10
        *   USARTx_CTS  : PA11
        *   USART1_RTS  : PA12
        * @}
        */
       
        /*USART1_TX   : PA9*/
        pinConfig.pinNumber = GPIO_PIN9;
        pinConfig.pinMode = GPIO_MODE_AF_OUTPUT_PP_10MHZ;
        MCAL_GPIO_Init(GPIOA, &pinConfig);

        /*USART1_RX   : PA10*/
        pinConfig.pinNumber = GPIO_PIN10;
        pinConfig.pinMode = GPIO_MODE_INPUT_FLOATING;
        MCAL_GPIO_Init(GPIOA, &pinConfig);

        /*USARTx_CTS  : PA11*/
        if(FlowCTRL == UART_FLOW_CTRL_CTS || FlowCTRL == UART_FLOW_CTRL_CTS_RTS)
        {
            pinConfig.pinNumber = GPIO_PIN11;
            pinConfig.pinMode = GPIO_MODE_INPUT_FLOATING;
            MCAL_GPIO_Init(GPIOA, &pinConfig);
        }

        /*USART1_RTS  : PA12*/
        if(FlowCTRL == UART_FLOW_CTRL_RTS || FlowCTRL == UART_FLOW_CTRL_CTS_RTS)
        {
            pinConfig.pinNumber = GPIO_PIN12;
            pinConfig.pinMode = GPIO_MODE_AF_OUTPUT_PP_10MHZ;
            MCAL_GPIO_Init(GPIOA, &pinConfig);
        }

    }else if(USART == USART2)
    {
        /** @defgroup USART2 Pins configuration
        * @{
        *   USART2_TX   : PA2
        *   USART2_RX   : PA3
        *   USARTx_CTS  : PA0
        *   USART2_RTS  : PA1
        * @}
        */
        
        /*USART2_TX   : PA2*/
        pinConfig.pinNumber = GPIO_PIN2;
        pinConfig.pinMode = GPIO_MODE_AF_OUTPUT_PP_10MHZ;
        MCAL_GPIO_Init(GPIOA, &pinConfig);

        /*USART2_RX   : PA3*/
        pinConfig.pinNumber = GPIO_PIN3;
        pinConfig.pinMode = GPIO_MODE_INPUT_FLOATING;
        MCAL_GPIO_Init(GPIOA, &pinConfig);

        /*USART2_CTS  : PA0*/
        if(FlowCTRL == UART_FLOW_CTRL_CTS || FlowCTRL == UART_FLOW_CTRL_CTS_RTS)
        {
            pinConfig.pinNumber = GPIO_PIN0;
            pinConfig.pinMode = GPIO_MODE_INPUT_FLOATING;
            MCAL_GPIO_Init(GPIOA, &pinConfig);
        }

        /*USART2_RTS  : PA1*/
        if(FlowCTRL == UART_FLOW_CTRL_RTS || FlowCTRL == UART_FLOW_CTRL_CTS_RTS)
        {
            pinConfig.pinNumber = GPIO_PIN1;
            pinConfig.pinMode = GPIO_MODE_AF_OUTPUT_PP_10MHZ;
            MCAL_GPIO_Init(GPIOA, &pinConfig);
        }

    }else if(USART == USART3)
    {
        /** @defgroup USART3 Pins configuration
        * @{
        *   USART3_TX   : PB10
        *   USART3_RX   : PB11
        *   USARTx_CTS  : PB13
        *   USART3_RTS  : PB14
        * @}
        */

        /*USART3_TX   : PB10*/
        pinConfig.pinNumber = GPIO_PIN10;
        pinConfig.pinMode = GPIO_MODE_AF_OUTPUT_PP_10MHZ;
        MCAL_GPIO_Init(GPIOB, &pinConfig);

        /*USART3_RX   : PB11*/
        pinConfig.pinNumber = GPIO_PIN11;
        pinConfig.pinMode = GPIO_MODE_INPUT_FLOATING;
        MCAL_GPIO_Init(GPIOB, &pinConfig);

        /*USART3_CTS  : PB13*/
        if(FlowCTRL == UART_FLOW_CTRL_CTS || FlowCTRL == UART_FLOW_CTRL_CTS_RTS)
        {
            pinConfig.pinNumber = GPIO_PIN13;
            pinConfig.pinMode = GPIO_MODE_INPUT_FLOATING;
            MCAL_GPIO_Init(GPIOB, &pinConfig);
        }

        /*USART3_RTS  : PB14*/
        if(FlowCTRL == UART_FLOW_CTRL_RTS || FlowCTRL == UART_FLOW_CTRL_CTS_RTS)
        {
            pinConfig.pinNumber = GPIO_PIN14;
            pinConfig.pinMode = GPIO_MODE_AF_OUTPUT_PP_10MHZ;
            MCAL_GPIO_Init(GPIOB, &pinConfig);
        }
    }
}

/**************************************************************************************************************************
*===============================================
*  			APIs functions definitions
*===============================================
*/

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_Init
* @brief		:   Intitialization of the specified UART instance using the specified parameters in the config structure.
* @param [in]	:   USART: specifies the UART instance to be initialized can be (USART1, USART2 or USART3).
* @param [in]	:   config: specifies the configuration parameters for the specified UART instance.
* @return_value :   none.
* Note			:   To enable any of the interrupts use the function "MCAL_UART_Interrupt_EN".
======================================================================================================================
*/
void MCAL_UART_Init(USART_Typedef * USART, UART_config_t* config)
{
    /*Enable the specified USART instance clock*/
    if(USART == USART1)
    {
        APB2_PERI_CLOCK_EN(APB2_USART1);
    }else if(USART == USART2)
    {
        APB1_PERI_CLOCK_EN(APB1_USART2);
    }else if(USART == USART3)
    {
        APB1_PERI_CLOCK_EN(APB1_USART3);
    }

    /*Enable the USART by writing the UE bit in USART_CR1 register to 1*/
    USART->CR1 |= USART_CR1_UE;

    /*Program the M bit in USART_CR1 to define the word length*/
    USART->CR1 &= ~(USART_CR1_M);           /*Clear the word length bit*/ 
    USART->CR1 |= config->wordLength;

    /*Program the number of stop bits in USART_CR2*/
    USART->CR2 &= ~(USART_CR2_STOP);        /*Clear the stop bits*/
    USART->CR2 |= config->stopBits;

    /*Program the parity in USART_CR1*/
    USART->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS); /*Clear the parity bits*/       
    USART->CR1 |= config->parityCTRL;
    
    /*Program the HW flow control in USART_CR3*/
    USART->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE); /*Clear the flow control bits*/
    USART->CR3 |= config->HW_FlowCTRL;

    /*Select the desired baud rate using the USART_BRR register*/
    if(USART == USART1)
    {
        USART->BRR = UART_BRR_VALUE(MCAL_RCC_GET_PCLK2(), config->baudRate);
    }else{
        USART->BRR = UART_BRR_VALUE(MCAL_RCC_GET_PCLK1(), config->baudRate);
    }

    /*Select the TX/RX operating mode*/
    USART->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
    USART->CR1 |= config->UART_Mode;

    /*Intialize the GPIO pins*/
    MCAL_UART_GPIO_Set_Pins(USART, config->HW_FlowCTRL);
}

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_RESET
* @brief		:   Reset the specified UART instance to its original state.
* @param [in]	:   USART: specifies the UART instance to be reset, can be (USART1, USART2 or USART3).
* @param [out]	:   none.
* @return_value :   none.
* Note			:   none.
======================================================================================================================
*/
void MCAL_UART_RESET(USART_Typedef * USART)
{
    if(USART == USART1)
    {
        APB2_PERI_RESET(APB2_USART1);
    }
    else if(USART == USART2)
    {
        APB1_PERI_RESET(APB1_USART2);
    }
    else if(USART == USART3)
    {
        APB1_PERI_RESET(APB1_USART3);
    }
}

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_Send_Data
* @brief		:   Sends data through the specified UART instance.
* @param [in]	:   USART: specifies the UART instance, can be (USART1, USART2 or USART3).
* @param [in]	:   pTxBuffer: pointer to the data to be sent.
* @param [in]	:   PollingEn: Whether to use the polling mechanism or not, must be set based on @ref UART_POLLING_DEFINE.
* @return_value :   none.
* Note			:   If the PCE bit is set in USART_CR1, then the MSB bit of the data
*               :   written in the data register is transmitted but is changed by the parity bit.
======================================================================================================================
*/
void MCAL_UART_Send_Data(USART_Typedef * USART, uint16* pTxBuffer, uint8 PollingEn)
{
    /*If the user is using polling mechanism wait untill the transmit data register is empty*/
    if(PollingEn == UART_POLLING_ENABLE)
    {
        while((USART->SR & USART_SR_TXE) == USART_TX_BUFFER_FULL);
    }


    /*Check if we are sending 8bit or 9bit data*/
    if((USART->CR1 & USART_CR1_M) == UART_DATA_8bits)
    {
        USART->DR = (*pTxBuffer & (uint8)0xFF);
    }else{
        USART->DR = (*pTxBuffer & (uint16)0x01FF);
    }
}

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_Get_Data
* @brief		:   Receives the data from the UART.
* @param [in]	:   USART: specifies the UART instance, can be (USART1, USART2 or USART3).
* @param [in]	:   PollingEn: Whether to use the polling mechanism or not, must be set based on @ref UART_POLLING_DEFINE.
* @param [out]	:   pRxBuffer: pointer to the buffer to receive the data in.
* @return_value :   none.
* Note			:   none.
======================================================================================================================
*/
void MCAL_UART_Get_Data(USART_Typedef * USART, uint16* pRxBuffer,  uint8 PollingEn)
{
    /*If the user is using polling mechanism wait until the receive data register is full*/
    if(PollingEn == UART_POLLING_ENABLE)
    {
        while((USART->SR & USART_SR_RXNE) == USART_RX_BUFFER_EMPTY);
    }

    /*Check if we are reading 8bit or 9bit data*/
    if((USART->CR1 & USART_CR1_M) == UART_DATA_8bits)
    {
        /*If the parity is disabled read the entire 8bits*/
        if((USART->CR1 & USART_CR1_PCE) == UART_PARITY_NONE)
        {
            *((uint16 *)pRxBuffer) = (USART->DR & (uint16)0xFF);

        }else{

            /*If the parity is enabled the MSB is replaced with the parity*/
            *((uint16 *)pRxBuffer) = (USART->DR & (uint16)0x7F);
        }

    }else{
        /*If the parity is disabled read the entire 9bits*/
        if((USART->CR1 & USART_CR1_PCE) == UART_PARITY_NONE)
        {
            *((uint16 *)pRxBuffer) = USART->DR;

        }else{

            /*If the parity is enabled the MSB is replaced with the parity*/
            *((uint16 *)pRxBuffer) = (USART->DR & (uint16)0xFF);

        }    
    }

}

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_WAIT_TC
* @brief		:   This function waits for the transmission of data from the UART to be completed.
* @param [in]	:   none.
* @param [out]	:   none.
* @return_value :   none.
* Note			:   none.
======================================================================================================================
*/
void MCAL_UART_WAIT_TC(USART_Typedef * USART)
{   
    /*Wait until the transmission complete flag is set*/
    while((USART->SR & USART_SR_TC) == USART_TRANS_NOT_COMPLETE);
}

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_Interrupt_EN
* @brief		:   Enabel a specific interrupt and set its callback function.
* @param [in]	:   USART: specifies the UART instance, can be (USART1, USART2 or USART3).
* @param [in]	:   IRQ: specifies the interrupt to be enabled, must be set based on @ref UART_IRQ_DEFINE.
* @param [in]	:   p_IRQ_callback: pointer to the callback function to be executed.
* @return_value :   none.
* Note			:   none.
======================================================================================================================
*/
void MCAL_UART_Interrupt_EN(USART_Typedef * USART, uint8 IRQ, void (* p_IRQ_callback)(void))
{
    /*Enable the USART instance interrupt in NVIC and set the callback function in the right array*/
    if(USART == USART1)
    {
        MCAL_NVIC_EnableIRQ(NVIC_USART1_IVT_INDEX);
        p_IRQ_callback_USART1[IRQ] = p_IRQ_callback;
    }
    else if(USART == USART2)
    {
        MCAL_NVIC_EnableIRQ(NVIC_USART2_IVT_INDEX);
        p_IRQ_callback_USART2[IRQ] = p_IRQ_callback;
    }
    else if(USART == USART3)
    {
        MCAL_NVIC_EnableIRQ(NVIC_USART3_IVT_INDEX);
        p_IRQ_callback_USART3[IRQ] = p_IRQ_callback;
    }

    if(IRQ == USART_IRQ_ERROR || IRQ == USART_IRQ_CTS)
    {
        SET_BIT(USART->CR3, IRQ);

    }else{

        SET_BIT(USART->CR1, IRQ);
    }
}

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_Interrupt_Disable
* @brief		:   Disable a specific interrupt
* @param [in]	:   USART: specifies the UART instance, can be (USART1, USART2 or USART3).
* @param [in]	:   IRQ: specifies the interrupt to be disabled, must be set based on @ref USART_IRQ_DEFINE.
* @return_value :   none.
* Note			:   This function won't disable the USART instance interrupt in NVIC you have to manually disable it.
======================================================================================================================
*/
void MCAL_UART_Interrupt_Disable(USART_Typedef * USART, uint8 IRQ)
{
    if(IRQ == USART_IRQ_ERROR || IRQ == USART_IRQ_CTS)
    {
        CLEAR_BIT(USART->CR3, IRQ);

    }else{

        CLEAR_BIT(USART->CR1, IRQ);
    }
}

/**************************************************************************************************************************
===============================================
*               USART ISR functions
*===============================================
*/

void USART1_IRQHandler(void)
{
    /*Check which flag caused the interrupt*/
    uint16 TempBuff;
    /*FE, NE, ORE have the same ISR*/
    if((USART1->SR & USART_SR_FE) || (USART1->SR & USART_SR_ORE) || ((USART1->SR & USART_SR_NE) &&
        (USART1->CR3 & USART_CR3_EIE)))
    {
        /*It is cleared by a software sequence (an read to the
            USART_SR register followed by a read to the USART_DR register)*/
        TempBuff = (uint16)USART1->DR;

        if(p_IRQ_callback_USART1[USART_IRQ_ERROR])
        {
            p_IRQ_callback_USART1[USART_IRQ_ERROR]();
        }
    }

    if((USART1->SR & USART_SR_IDLE) && (USART1->CR1 & USART_CR1_IDLEIE))
    {
        /*It is cleared by a software sequence (an read to the
            USART_SR register followed by a read to the USART_DR register)*/
        TempBuff = (uint16)USART1->DR;

        if(p_IRQ_callback_USART1[USART_IRQ_IDLE])
        {
            p_IRQ_callback_USART1[USART_IRQ_IDLE]();
        }
    }

    if((USART1->SR & USART_SR_RXNE) && (USART1->CR1 & USART_CR1_RXNEIE))
    {
        /*It is cleared by a read to the USART_DR register, "Must be done by the User".*/
        if(p_IRQ_callback_USART1[USART_IRQ_RXNE])
        {
            p_IRQ_callback_USART1[USART_IRQ_RXNE]();
        }
    }

    if((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
    {
        /*It is cleared by a write to the USART_DR register, "Must be done by the User".*/
        if(p_IRQ_callback_USART1[USART_IRQ_TXE])
        {
            p_IRQ_callback_USART1[USART_IRQ_TXE]();
        }
    }

    if((USART1->SR & USART_SR_TC) && (USART1->CR1 & USART_CR1_TCIE))
    {
        /*It is cleared by a software sequence (a read from the USART_SR register
         followed by a write to the USART_DR register), 
         The TC bit can also be cleared by writing a '0' to it*/
        USART1->SR &= ~(USART_SR_TC);
               
        if(p_IRQ_callback_USART1[USART_IRQ_TC])
        {
           p_IRQ_callback_USART1[USART_IRQ_TC]();
        }
    }

    if((USART1->SR & USART_SR_PE) && (USART1->CR1 & USART_CR1_PEIE))
    {
        /*It is cleared by a software sequence (a read from the USART_SR register
         followed by a read from the USART_DR register)*/
        TempBuff =  USART1->DR;
        
        if( p_IRQ_callback_USART1[USART_IRQ_PE])
        {
            p_IRQ_callback_USART1[USART_IRQ_PE]();
        }
    }

    if((USART1->SR & USART_SR_CTS) && (USART1->CR3 & USART_CR3_CTSIE))
    {
        /*It is cleared by software (by writing it to 0).*/
        USART1->SR &= ~(USART_SR_CTS);

        if(p_IRQ_callback_USART1[USART_IRQ_CTS])
        {
            p_IRQ_callback_USART1[USART_IRQ_CTS]();
        }
    }
}

void USART2_IRQHandler(void)
{
    /*Check which flag caused the interrupt*/
    uint16 TempBuff;
    /*FE, NE, ORE have the same ISR*/
    if((USART2->SR & USART_SR_FE) || (USART2->SR & USART_SR_ORE) || (USART2->SR & USART_SR_NE) &&
        (USART2->CR3 & USART_CR3_EIE))
    {
        /*It is cleared by a software sequence (an read to the
            USART_SR register followed by a read to the USART_DR register)*/
        TempBuff = (uint16)USART2->DR;

        if(p_IRQ_callback_USART2[USART_IRQ_ERROR])
        {
            p_IRQ_callback_USART2[USART_IRQ_ERROR]();
        }
    }

    if((USART2->SR & USART_SR_IDLE) && (USART2->CR1 & USART_CR1_IDLEIE))
    {
        /*It is cleared by a software sequence (an read to the
            USART_SR register followed by a read to the USART_DR register)*/
        TempBuff = (uint16)USART2->DR;

        if(p_IRQ_callback_USART2[USART_IRQ_IDLE])
        {
            p_IRQ_callback_USART2[USART_IRQ_IDLE]();
        }
    }

    if((USART2->SR & USART_SR_RXNE) && (USART2->CR1 & USART_CR1_RXNEIE))
    {
        /*It is cleared by a read to the USART_DR register, "Must be done by the User".*/
        if(p_IRQ_callback_USART2[USART_IRQ_RXNE])
        {
            p_IRQ_callback_USART2[USART_IRQ_RXNE]();
        }
    }

    if((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))
    {
        /*It is cleared by a write to the USART_DR register, "Must be done by the User".*/
        if(p_IRQ_callback_USART2[USART_IRQ_TXE])
        {
            p_IRQ_callback_USART2[USART_IRQ_TXE]();
        }
    }

    if((USART2->SR & USART_SR_TC) && (USART2->CR1 & USART_CR1_TCIE))
    {
        /*It is cleared by a software sequence (a read from the USART_SR register
         followed by a write to the USART_DR register), 
         The TC bit can also be cleared by writing a '0' to it*/
        USART2->SR &= ~(USART_SR_TC);
        
        if(p_IRQ_callback_USART2[USART_IRQ_TC])
        {
           p_IRQ_callback_USART2[USART_IRQ_TC]();
        }
    }

    if((USART2->SR & USART_SR_PE) && (USART2->CR1 & USART_CR1_PEIE))
    {
        /*It is cleared by a software sequence (a read from the USART_SR register
         followed by a read from the USART_DR register)*/
        TempBuff =  USART2->DR;
        
        if( p_IRQ_callback_USART2[USART_IRQ_PE])
        {
            p_IRQ_callback_USART2[USART_IRQ_PE]();
        }
    }

    if((USART2->SR & USART_SR_CTS) && (USART2->CR3 & USART_CR3_CTSIE))
    {
        /*It is cleared by software (by writing it to 0).*/
        USART2->SR &= ~(USART_SR_CTS);

        if(p_IRQ_callback_USART2[USART_IRQ_CTS])
        {
            p_IRQ_callback_USART2[USART_IRQ_CTS]();
        }
    }
}

void USART3_IRQHandler(void)
{
    /*Check which flag caused the interrupt*/
    uint16 TempBuff;
    /*FE, NE, ORE have the same ISR*/
    if((USART3->SR & USART_SR_FE) || (USART3->SR & USART_SR_ORE) || (USART3->SR & USART_SR_NE) &&
        (USART3->CR3 & USART_CR3_EIE))
    {
        /*It is cleared by a software sequence (an read to the
            USART_SR register followed by a read to the USART_DR register)*/
        TempBuff = (uint16)USART3->DR;

        if(p_IRQ_callback_USART3[USART_IRQ_ERROR])
        {
            p_IRQ_callback_USART3[USART_IRQ_ERROR]();
        }
    }

    if((USART3->SR & USART_SR_IDLE) && (USART3->CR1 & USART_CR1_IDLEIE))
    {
        /*It is cleared by a software sequence (an read to the
            USART_SR register followed by a read to the USART_DR register)*/
        TempBuff = (uint16)USART3->DR;

        if(p_IRQ_callback_USART3[USART_IRQ_IDLE])
        {
            p_IRQ_callback_USART3[USART_IRQ_IDLE]();
        }
    }

    if((USART3->SR & USART_SR_RXNE) && (USART3->CR1 & USART_CR1_RXNEIE))
    {
        /*It is cleared by a read to the USART_DR register, "Must be done by the User".*/
        if(p_IRQ_callback_USART3[USART_IRQ_RXNE])
        {
            p_IRQ_callback_USART3[USART_IRQ_RXNE]();
        }
    }

    if((USART3->SR & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE))
    {
        /*It is cleared by a write to the USART_DR register, "Must be done by the User".*/
        if(p_IRQ_callback_USART3[USART_IRQ_TXE])
        {
            p_IRQ_callback_USART3[USART_IRQ_TXE]();
        }
    }

    if((USART3->SR & USART_SR_TC) && (USART3->CR1 & USART_CR1_TCIE))
    {
        /*It is cleared by a software sequence (a read from the USART_SR register
         followed by a write to the USART_DR register), 
         The TC bit can also be cleared by writing a '0' to it*/
        USART3->SR &= ~(USART_SR_TC);
        
        if(p_IRQ_callback_USART3[USART_IRQ_TC])
        {
           p_IRQ_callback_USART3[USART_IRQ_TC]();
        }
    }

    if((USART3->SR & USART_SR_PE) && (USART3->CR1 & USART_CR1_PEIE))
    {
        /*It is cleared by a software sequence (a read from the USART_SR register
         followed by a read from the USART_DR register)*/
        TempBuff =  USART3->DR;
        
        if( p_IRQ_callback_USART3[USART_IRQ_PE])
        {
            p_IRQ_callback_USART3[USART_IRQ_PE]();
        }
    }

    if((USART3->SR & USART_SR_CTS) && (USART3->CR3 & USART_CR3_CTSIE))
    {
        /*It is cleared by software (by writing it to 0).*/
        USART3->SR &= ~(USART_SR_CTS);

        if(p_IRQ_callback_USART3[USART_IRQ_CTS])
        {
            p_IRQ_callback_USART3[USART_IRQ_CTS]();
        }
    }
}
