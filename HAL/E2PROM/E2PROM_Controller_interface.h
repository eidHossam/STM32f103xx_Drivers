/*
 **************************************************************************************************************************
 * file		 :  E2PROM_interface.h
 * brief     :  
 * Author    :  Hossam Eid
 * Created on:  Nov 22, 2023
 **************************************************************************************************************************
 * Description:
 * 
 *
 * note:
 **************************************************************************************************************************
 */
#ifndef HAL_E2PROM_E2PROM_CONTROLLER_INTERFACE_H_
#define HAL_E2PROM_E2PROM_CONTROLLER_INTERFACE_H_

#include "I2C/stm32f103x6_I2C.h"

/** @defgroup E2PROM Defines
  * @{
  */
#define E2PROM_Slave_address	0x2A
/**
  * @}
  */


/** @defgroup E2PROM type definitions
  * @{
  */
typedef enum{
  E2PROM_DataSizeLimitExceeded,
  E2PROM_TransmissionComplete
}eStatus;
/**
  * @}
  */

/**************************************************************************************************************************
================================================
*       APIs Supported by "HAL E2PROM DRIVER"
*===============================================
*/

/*
======================================================================================================================
* @Func_name	:   E2PROM_Init
* @brief		:   Initializes the E2PROM driver subsystem.
======================================================================================================================
*/
void E2PROM_Init(void);

/*
======================================================================================================================
* @Func_name	:   E2PROM_write_Nbytes
* @brief		:   Writes a number of bytes to the E2PROM memory.
* @param [in]	:   Memory_address: The address of the E2PROM memory to write the data at.
* @param [in]	:   buffer: Buffer storing the data to be written on the E2PROM memory.
* @param [in]	:   Data_Length: Number of bytes to be stored in the E2PROM memory.
======================================================================================================================
*/
eStatus E2PROM_write_Nbytes(uint16 Memory_address, uint8* buffer, uint8 Data_Length);

/*
======================================================================================================================
* @Func_name	:   E2PROM_read_nbytes
* @brief		:   Read a number of bytes from E2PROM memory.
* @param [in]	:   Memory_address: The address of the E2PROM memory to read the data from.
* @param [in]	:   Data_Length: Number of bytes to be stored in the E2PROM memory.
* @param [out]	:   dataOut: Buffer to store the data.
======================================================================================================================
*/
eStatus E2PROM_read_nbytes(uint16 Memory_address , uint8* dataOut, uint8 Data_Length);


#endif /* HAL_E2PROM_E2PROM_CONTROLLER_INTERFACE_H_ */
