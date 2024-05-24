/*
 **************************************************************************************************************************
 * file		 : E2PROM_program.c
 * brief     : 
 * Author    : Hossam Eid
 * Created on: Nov 22, 2023
 **************************************************************************************************************************
 * Description:
 * 
 *
 * note:
 **************************************************************************************************************************
 */

/*
*===============================================
*                   Includes
*===============================================
*/
#include "E2PROM_Controller_interface.h"

/**************************************************************************************************************************
*===============================================
*  			APIs functions definitions
*===============================================
*/

/*
======================================================================================================================
* @Func_name	:   E2PROM_Init
* @brief		:   Initializes the E2PROM driver subsystem.
======================================================================================================================
*/
void E2PROM_Init(void)
{
 	I2C_CFG_t cfg;

    /*I2C Controller act as a Master*/
	cfg.I2C_Mode = I2C_MODE_MASTER;
	cfg.I2C_ClockSpeed = I2C_CLKSPEED_SM100KHZ;
	cfg.I2C_Ack = I2C_ACK_ENABLE;
	cfg.Slave_ClockStretchEN = I2C_CLKSTRETCH_ENABLE;
	cfg.Slave_GeneralCallEN = I2C_GCALL_ENABLE;	

	MCAL_I2C_Init(I2C1, &cfg);   
}

/*
======================================================================================================================
* @Func_name	:   E2PROM_write_Nbytes
* @brief		:   Writes a number of bytes to the E2PROM memory.
* @param [in]	:   Memory_address: The address of the E2PROM memory to write the data at.
* @param [in]	:   buffer: Buffer storing the data to be written on the E2PROM memory.
* @param [in]	:   Data_Length: Number of bytes to be stored in the E2PROM memory.
======================================================================================================================
*/
eStatus E2PROM_write_Nbytes(uint16 Memory_address, uint8* buffer, uint8 Data_Length)
{	
	/*Make a new buffer holding the address + the data*/
    uint8 newBuffer[256], i;
	I2C_MasterTxRx_CFG_t Buffer_CFG;

	if(Data_Length >= 254)
		return E2PROM_DataSizeLimitExceeded;
	
	newBuffer[0] = (uint8)(Memory_address >> 8); 	/*Store the most significant byte of the address*/
	newBuffer[1] = (uint8)(Memory_address);			/*Store the most significant byte of the address*/

	/*Copy the data to the new buffer*/
	for(i = 2; i < (Data_Length + 2); i++)
	{
		newBuffer[i] = buffer[i - 2];
	}

	/*Configure the transmission parameters*/
	Buffer_CFG.bufferSize = Data_Length + 2;
	Buffer_CFG.slaveAddress = E2PROM_Slave_address;
	Buffer_CFG.startCondition = NormalStartCondition;
	Buffer_CFG.stopCondition = StopConditionEN;
	Buffer_CFG.TX_dataBuffer = newBuffer;
	
	/*Send the data on the I2C Bus*/
	MCAL_I2C_MasterTX(I2C1, &Buffer_CFG);

	return E2PROM_TransmissionComplete;
}

/*
======================================================================================================================
* @Func_name	:   E2PROM_read_nbytes
* @brief		:   Read a number of bytes from E2PROM memory.
* @param [in]	:   Memory_address: The address of the E2PROM memory to read the data from.
* @param [in]	:   Data_Length: Number of bytes to be stored in the E2PROM memory.
* @param [out]	:   dataOut: Buffer to store the data.
======================================================================================================================
*/
eStatus E2PROM_read_nbytes(uint16 Memory_address , uint8* dataOut, uint8 Data_Length)
{
    /*Make a new buffer holding the address to be read*/
	I2C_MasterTxRx_CFG_t Buffer_CFG;
    uint8 newBuffer[2];
	
	newBuffer[0] = (uint8)(Memory_address >> 8); 	/*Store the most significant byte of the address*/
	newBuffer[1] = (uint8)(Memory_address);			/*Store the most significant byte of the address*/

	/*Configure the transmission parameters*/
	Buffer_CFG.bufferSize = 2;
	Buffer_CFG.slaveAddress = E2PROM_Slave_address;
	Buffer_CFG.startCondition = NormalStartCondition;
	Buffer_CFG.stopCondition = StopConditionDIS;
	Buffer_CFG.TX_dataBuffer = newBuffer;
	
	/*Send the data on the I2C Bus*/
	MCAL_I2C_MasterTX(I2C1, &Buffer_CFG);

	/*Configure the Reception parameters*/
	Buffer_CFG.bufferSize = Data_Length;
	Buffer_CFG.slaveAddress = E2PROM_Slave_address;
	Buffer_CFG.startCondition = RepeatedStartCondition;
	Buffer_CFG.stopCondition = StopConditionEN;
	Buffer_CFG.TX_dataBuffer = dataOut;
	
	/*Receive the data from the E2PROM using the I2C Bus*/
	MCAL_I2C_MasterRX(I2C1, &Buffer_CFG);

	return E2PROM_TransmissionComplete;
}

