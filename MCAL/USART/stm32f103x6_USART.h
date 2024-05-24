/*
**************************************************************************************************************************
* file		: stm32f103x6_USART.h
* brief     : This files contains functions to configure the USART communication module.
* Author    : Hossam Eid
* Created on: Sep 19, 2023
**************************************************************************************************************************
* note: This module is intended for use with the STM32F103xx microcontroller series, but may be adapted for use with
* other compatible processors.
**************************************************************************************************************************
*/

#ifndef MCAL_USART_STM32F103X6_USART_H_
#define MCAL_USART_STM32F103X6_USART_H_

/*
*===============================================
*                   Includes
*===============================================
*/

#include "stm32f103x6.h"
#include "Platform_Types.h"
#include "GPIO/stm32f103x6_GPIO.h"
#include "NVIC/stm32f103x6_NVIC.h"
#include "RCC/stm32f103x6_RCC.h"

/**************************************************************************************************************************
*===============================================
* User type definitions (structures)
*===============================================
*/
typedef struct{
    uint8       UART_Mode;     /*Specifies the operating mode of the UART (TX only- RX only - Both)
                                must be set based on @ref UART_MODE_DEFINE*/

    uint32      baudRate;       /*Specifies the operating baud rate of the UART 
                                  must be set based on @ref UART_BAUD_RATE_DEFINE*/ 

    uint32       wordLength;     /*Specifies the data word length of the UART payload 
                                  must be set based on @ref UART_DATA_DEFINE*/                            

    uint32       stopBits;       /*Specifies the number of stop bits of the UART frame 
                                  must be set based on @ref UART_STOP_BITS_DEFINE*/                            
    
    uint32       parityCTRL;     /*Specifies the parity of the UART frame 
                                  must be set based on @ref UART_PARITY_DEFINE*/                            

    uint32       HW_FlowCTRL;     /*Specifies flow control (CTS - RTS) of the UART  
                                  must be set based on @ref UART_FLOW_CTRL_DEFINE*/                            
}UART_config_t;

/**************************************************************************************************************************
*===============================================
*         Macros Configuration References
*===============================================
*/

/*-----------@ref UART_MODE_DEFINE----------*/
#define UART_MODE_TX        USART_CR1_TE
#define UART_MODE_RX        USART_CR1_RE
#define UART_MODE_TX_RX     (USART_CR1_TE | USART_CR1_RE)

/*----------- @ref UART_BAUD_RATE_DEFINE----------*/
#define UART_BAUD_RATE_2400             2400
#define UART_BAUD_RATE_9600             9600
#define UART_BAUD_RATE_19200            19200
#define UART_BAUD_RATE_57600            57600
#define UART_BAUD_RATE_115200           115200
#define UART_BAUD_RATE_230400           230400
#define UART_BAUD_RATE_460800           460800
#define UART_BAUD_RATE_921600           921600
#define UART_BAUD_RATE_2250000          2250000
#define UART_BAUD_RATE_4500000          4500000

/*----------- @ref UART_DATA_DEFINE ----------*/
#define UART_DATA_8bits     0
#define UART_DATA_9bits     USART_CR1_M

/*----------- @ref UART_STOP_BITS_DEFINE ----------*/
/*STOP bits:
These bits are used for programming the stop bits.
00: 1 stop bit: This is the default value.
01: 0.5 stop bit: To be used when receiving data in Smartcard mode.
10: 2 stop bits: This is supported by normal USART, single-wire and modem modes.
11: 1.5 stop bits: To be used when transmitting and receiving data in Smartcard mode.
*/
#define UART_STOP_BITS_1            0  
#define UART_STOP_BITS_haf          (0b01 << 12)
#define UART_STOP_BITS_2            (0b10 << 12)
#define UART_STOP_BITS_1_half       (0b11 << 12)

/*----------- @ref UART_PARITY_DEFINE ----------*/
/*Note: If the PCE bit is set in USART_CR1, then the MSB bit of the data
written in the data register is transmitted but is changed by the parity bit*/
#define UART_PARITY_NONE            0
#define UART_PARITY_EVEN            USART_CR1_PCE
#define UART_PARITY_ODD             (USART_CR1_PCE | USART_CR1_PS)

/*----------- @ref UART_FLOW_CTRL_DEFINE ----------*/
#define UART_FLOW_CTRL_NONE          0
#define UART_FLOW_CTRL_CTS          USART_CR3_CTSE
#define UART_FLOW_CTRL_RTS          USART_CR3_RTSE
#define UART_FLOW_CTRL_CTS_RTS      (USART_CR3_CTSE | USART_CR3_RTSE)

/*----------- @ref UART_POLLING_DEFINE ----------*/
#define UART_POLLING_ENABLE     1
#define UART_POLLING_DISABLE    0

/*----------- @ref USART_IRQ_DEFINE ----------*/
#define USART_IRQ_ERROR         0
#define USART_IRQ_IDLE          4
#define USART_IRQ_RXNE          5
#define USART_IRQ_TC            6
#define USART_IRQ_TXE           7
#define USART_IRQ_PE            8
#define USART_IRQ_CTS           10

/**************************************************************************************************************************
*===============================================
*         Control/Status Registers Pins Macros
*===============================================
*/

/*
*  USART control register 1 (USART_CR1) bit definitions.
*/
#define USART_CR1_UE        0x00002000      /* USART enable */
#define USART_CR1_M         0x00001000      /* Word length */
#define USART_CR1_PCE       0x00000400      /* Parity control enable */
#define USART_CR1_PS        0x00000200      /* Parity selection */
#define USART_CR1_PEIE      0x00000100      /* PE interrupt enable */
#define USART_CR1_TXEIE     0x00000080      /* TXE interrupt enable */
#define USART_CR1_TCIE      0x00000040      /* Transmission complete interrupt enable */
#define USART_CR1_RXNEIE    0x00000020      /* RXNE interrupt enable */
#define USART_CR1_IDLEIE    0x00000010     /* IDLEIE interrupt enable */
#define USART_CR1_TE        0x00000008      /* Transmitter enable */
#define USART_CR1_RE        0x00000004      /* Receiver enable */

/*
*  USART control register 2 (USART_CR2) bit definitions.
*/
#define USART_CR2_STOP      0x00003000      /* STOP bits */

/*
*  USART control register 3 (USART_CR3) bit definitions.
*/
#define USART_CR3_CTSIE     0x00000400      /* CTS interrupt enable */
#define USART_CR3_CTSE      0x00000200      /* CTS enable */
#define USART_CR3_RTSE      0x00000100      /* RTS enable */
#define USART_CR3_EIE       0x00000001      /* Error interrupt enable */

/*
*  USART Status Register (USART_SR) bit definitions.
*/
#define USART_SR_CTS            0x00000200      /*CTS flag, This bit is set by hardware when the CTS input toggles*/  
#define USART_SR_TXE            0x00000080      /*Transmit data register empty flag*/  
#define USART_SR_TC             0x00000040      /*Transmission complete flag*/  
#define USART_SR_RXNE           0x00000020      /*Read data register not empty flag*/  
#define USART_SR_IDLE           0x00000010      /*IDLE line detected flag*/  
#define USART_SR_ORE            0x00000008      /*Overrun error flag*/  
#define USART_SR_NE             0x00000004      /*Noise error flag*/  
#define USART_SR_FE             0x00000002      /*Framing error flag*/  
#define USART_SR_PE             0x00000001      /*Parity error flag*/

#define USART_TX_BUFFER_EMPTY       1
#define USART_TX_BUFFER_FULL        0

#define USART_RX_BUFFER_EMPTY       0
#define USART_RX_BUFFER_FULL        1

#define USART_TRANS_NOT_COMPLETE    0

/**************************************************************************************************************************
*===============================================
*         Baud Rate Calculations Macros
*===============================================
*/
#define USARTDIV(CLK, baudRate)             ((uint32)(CLK / (16 * baudRate)))
#define USARTDIV_MUL100(CLK, baudRate)      ((uint32)((25 * CLK) / (4 * baudRate)))

#define DIV_Mantissa(CLK, baudRate)         ((uint32)USARTDIV(CLK, baudRate))
#define DIV_Mantissa_MUL100(CLK, baudRate)  ((uint32)(DIV_Mantissa(CLK, baudRate) * 100))

#define DIV_Fraction(CLK, baudRate)         ((uint32)((USARTDIV_MUL100(CLK, baudRate) - \
                                                        DIV_Mantissa_MUL100(CLK, baudRate)) * 16 )/ 100)

#define UART_BRR_VALUE(CLK, baudRate)       ((DIV_Mantissa(CLK, baudRate) << 4) | (DIV_Fraction(CLK,baudRate) & 0xF))




/**************************************************************************************************************************
===============================================
*       APIs Supported by "MCAL USART DRIVER"
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
void MCAL_UART_Init(USART_Typedef * USART, UART_config_t* config);

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
void MCAL_UART_RESET(USART_Typedef * USART);

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
void MCAL_UART_Send_Data(USART_Typedef * USART, uint16* pTxBuffer, uint8 PollingEn);

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
void MCAL_UART_Get_Data(USART_Typedef * USART, uint16* pRxBuffer,  uint8 PollingEn);

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
void MCAL_UART_WAIT_TC(USART_Typedef * USART);

/*
======================================================================================================================
* @Func_name	:   MCAL_UART_Interrupt_EN
* @brief		:   Enable a specific interrupt and set its callback function.
* @param [in]	:   USART: specifies the UART instance, can be (USART1, USART2 or USART3).
* @param [in]	:   IRQ: specifies the interrupt to be enabled, must be set based on @ref USART_IRQ_DEFINE.
* @param [in]	:   p_IRQ_callback: pointer to the callback function to be executed.
* @return_value :   none.
* Note			:   none.
======================================================================================================================
*/
void MCAL_UART_Interrupt_EN(USART_Typedef * USART, uint8 IRQ, void (* p_IRQ_callback)(void));

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
void MCAL_UART_Interrupt_Disable(USART_Typedef * USART, uint8 IRQ);

#endif /* MCAL_USART_STM32F103X6_USART_H_ */
