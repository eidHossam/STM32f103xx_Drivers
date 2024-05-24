/*
**************************************************************************************************************************
* file		: stm32f103x6_I2C.h
* brief     : This files contains functions to configure the I2C communication module.
* Author    : Hossam Eid
* Created on: Nov 20, 2023
**************************************************************************************************************************
* note: This module is intended for use with the STM32F103xx microcontroller series, but may be adapted for use with
* other compatible processors.
**************************************************************************************************************************
*/

#ifndef MCAL_STM32F103X6_I2C_H_
#define MCAL_STM32F103X6_I2C_H_

/*
*===============================================
*                   Includes
*===============================================
*/

#include "stm32f103x6.h"
#include "GPIO/stm32f103x6_GPIO.h"
#include "NVIC/stm32f103x6_NVIC.h"
#include "RCC/stm32f103x6_RCC.h"

/**************************************************************************************************************************
*===============================================
* User type definitions 
*===============================================
*/

/** @defgroup I2C_SLAVE_STATE_DEFINE
  * @{
  */

typedef enum
{
	I2C_EV_STOP,
	I2C_ERROR_AF ,
	I2C_EV_ADDR_Matched,
	I2C_EV_DATA_REQ,      /*the APP layer should send the data (I2C_SlaveSendData ) in this state*/
	I2C_EV_DATA_RCV       /*the APP layer should read the data (I2C_SlaveReceiveData ) in this state*/
}eSlave_State;

/**
  * @}
  */


typedef struct{
    uint8  I2C_SlaveDualAddressEn;  /*!<Enable or disable I2C dual slave address
                                         must be set based on @ref I2C_ENDUAL_DEFINE*/ 
    
    uint16  I2C_PrimarySlaveAddress;

    uint8  I2C_SecSlaveAddress;
    
    uint16 I2C_SlaveAddressLength;  /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                         must be set based on @ref I2C_SLAVEADDRESS_DEFINE */
}I2C_SLAVEADD_CFG_t;

typedef struct{
    uint16 I2C_Mode;                /*!< Choose between master or slave mode
                                         must be set based on @ref I2C_MODE_DEFINE*/

    uint32 I2C_ClockSpeed;          /*!< Specifies the clock frequency.
                                         This parameter must be set to a value lower than 100KHZ */

    uint16 I2C_Ack;                 /*Do or don't return an acknowledgement after reception
                                         must be set based on @ref I2C_ACK_DEFINE*/

    I2C_SLAVEADD_CFG_t I2C1_SLAVEADD_CFG;

    uint8 Slave_GeneralCallEN;      /*Enable or disable the slave general call mechanism
                                         must be set based on @ref I2C_GCALL_DEFINE*/

    uint8 Slave_ClockStretchEN;      /*Enable or disable the slave clock stretching mechanism
                                         must be set based on @ref I2C_NOSTRETCH_DEFINE*/

    void(* p_I2C_ISR_callback)(eSlave_State state);
}I2C_CFG_t;


/** @defgroup @ref I2C_StopCondition
  * @{
  */

typedef enum{
    StopConditionEN = 1,    /*Default*/
    StopConditionDIS = 0
}eI2C_StopCondition;

/**
  * @}
  */

/** @defgroup  @ref I2C_StartCondition
  * @{
  */

typedef enum{
    NormalStartCondition,   /*Default*/
    RepeatedStartCondition
}eI2C_StartCondition;

/**
  * @}
  */

typedef struct{
    uint16              slaveAddress;                   /*Address of the slave to send the data to.*/
    
    uint8*              TX_dataBuffer;                  /*Pointer to the buffer holding the data to be sent.*/
    
    uint8               bufferSize;                     /*Number of elements to send.*/
    
    eI2C_StopCondition  stopCondition;                  /*Choose if you want to send a stop condition or not.
                                                            must be set based on @ref I2C_StopCondition*/

    eI2C_StartCondition startCondition;                 /*Choose the type of the start condition.
                                                            must be set based on @ref I2C_StartCondition*/

}I2C_MasterTxRx_CFG_t;


/** @defgroup @ref I2C_EVENT_STATUS
  * @{
  */

typedef enum{Reset = 0, Set = 1}EventStatus;

/**
  * @}
  */


/** @defgroup @ref I2C_EVENTS
   * @{
   */

typedef enum{
    I2C_LineBusy,
    I2C_EV5,
    I2C_EV6,
    I2C_EV7,
    I2C_EV8,
    I2C_EV8_1,
    I2C_EV8_2 = (I2C_SR1_TXE | I2C_SR1_BTF),
    I2C_MasterTransmitting = (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)
}eEvent;

/**
   * @}
   */

/** @defgroup @ref I2C_DIRECTION_DEFINE
   * @{
   */

typedef enum{
    I2C_DirectionTransmission = 0,
    I2C_DirectionReception = 1
}eI2C_Direction;

/**
   * @}
   */


/**************************************************************************************************************************
*===============================================
*         Macros Configuration References
*===============================================
*/

/*-----------@ref PERIPEHAL_MODE_DEFINE----------*/
#define PERIPEHAL_MODE_I2C          0 
#define PERIPEHAL_MODE_SMBUS        I2C_CR1_SMBUS 

/*-----------@ref  I2C_MODE_DEFINE----------*/
#define  I2C_MODE_MASTER            0
#define  I2C_MODE_SLAVE             (I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN)

/*-----------@ref  I2C_GCALL_DEFINE----------*/
#define I2C_GCALL_ENABLE            I2C_CR1_ENGC
#define I2C_GCALL_DISABLE           0

/*-----------@ref  I2C_NOSTRETCH_DEFINE----------*/
#define I2C_CLKSTRETCH_ENABLE        0
#define I2C_CLKSTRETCH_DISABLE       I2C_CR1_NOSTRETCH

/*-----------@ref  I2C_ACK_DEFINE----------*/
#define I2C_ACK_ENABLE              I2C_CR1_ACK
#define I2C_ACK_DISABLE             0

/*-----------@ref  I2C_CLKSPEED_DEFINE----------*/
/*Note: Fast Mode isn't supported*/
#define I2C_CLKSPEED_SM10KHZ            10000UL
#define I2C_CLKSPEED_SM20KHZ            20000UL
#define I2C_CLKSPEED_SM30KHZ            30000UL
#define I2C_CLKSPEED_SM40KHZ            40000UL
#define I2C_CLKSPEED_SM50KHZ            50000UL
#define I2C_CLKSPEED_SM60KHZ            60000UL
#define I2C_CLKSPEED_SM70KHZ            70000UL
#define I2C_CLKSPEED_SM80KHZ            80000UL
#define I2C_CLKSPEED_SM90KHZ            90000UL
#define I2C_CLKSPEED_SM100KHZ           100000UL

/*-----------@ref  I2C_SLAVEADDRESS_DEFINE----------*/
#define I2C_SLAVEADDRESS_7BIT           0
#define I2C_SLAVEADDRESS_10BIT          (1 << 15) 

/*-----------@ref  I2C_ENDUAL_DEFINE----------*/
#define I2C_ENDUAL_ENABLE               I2C_OAR2_ENDUAL
#define I2C_ENDUAL_DISABLE              0


/**************************************************************************************************************************
===============================================
*       APIs Supported by "MCAL I2C DRIVER"
*===============================================
*/

/*
======================================================================================================================
* @Func_name	  :   MCAL_I2C_Init
* @brief		    :   Intitialization of the specified I2C instance using the specified parameters in the config structure.
* @param [in]	  :   I2Cx: specifies the I2C instance to be initialized, can be (I2C1, I2C2).
* @param [in]	  :   I2C_CFG: specifies the configuration parameters for the specified I2C instance.
* @return_value :   none.
======================================================================================================================
*/
void MCAL_I2C_Init(I2C_Typedef* I2Cx, I2C_CFG_t* I2C_CFG);

/*
======================================================================================================================
* @Func_name	  :   MCAL_I2C_Reset
* @brief		    :   Reset the specified UART instance to its original state.
* @param [in]	  :   I2Cx: specifies the I2C instance to be reset, can be (I2C1, I2C2).
* @param [out]	:   none.
* @return_value :   none.
* Note			    :   none.
======================================================================================================================
*/
void MCAL_I2C_Reset(I2C_Typedef* I2Cx);

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_MasterTX
* @brief		  :   Function to send data from a master to a slave on the I2C bus.
* @param [in]	:   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	:   masterTxRx_cfg: A struct to configure the transmission options for the master.
* @note       :   This functions works with a polling mechanism, a timeout, interrupt mechanism is to be added.
======================================================================================================================
*/
void MCAL_I2C_MasterTX(I2C_Typedef* I2Cx, I2C_MasterTxRx_CFG_t* masterTxRx_cfg);

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_MasterRX
* @brief		  :   Function to receive data from a slave as the master on the I2C bus.
* @param [in]	:   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	:   masterTxRx_cfg: A struct to configure the transmission options for the master.
* @note       :   This functions works with a polling mechanism, a timeout, interrupt mechanism is to be added.
======================================================================================================================
*/
void MCAL_I2C_MasterRX(I2C_Typedef* I2Cx, I2C_MasterTxRx_CFG_t* masterTxRx_cfg);

/*
======================================================================================================================
* @Func_name	  : MCAL_I2C_SlaveTx
* @brief		    : This function sends data on the I2C bus as a slave when called by the master
* @param [in]	  : I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [out]	: packet: data to be sent.
======================================================================================================================
*/
void MCAL_I2C_SlaveTx(I2C_Typedef* I2Cx, uint8 packet);

/*
======================================================================================================================
* @Func_name	  : MCAL_I2C_SlaveRx
* @brief		    : This function receives data from the I2C bus as a slave when called by the master
* @param [in]	  : I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @return_value : the received data.
======================================================================================================================
*/
uint8 MCAL_I2C_SlaveRx(I2C_Typedef* I2Cx);

/*
======================================================================================================================
* @Func_name	  :   MCAL_I2C_GenerateStart
* @brief		    :   Function to generate the start condition for the I2C bus based on the passed parameters.
* @param [in]	  :   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	  :   startCondition: Choose the type of the start condition must be set based on @ref I2C_StartCondition.
* @param [in]	  :   NewState: The new state to be set for the start condition must be a value of {Enable, Disable};
* @return_value :   none.
======================================================================================================================
*/
void MCAL_I2C_GenerateStart(I2C_Typedef* I2Cx, eI2C_StartCondition startCondition, FunctionalState NewState);

/*
======================================================================================================================
* @Func_name	  :   MCAL_I2C_GenerateStop
* @brief		    :   Function to generate the Stop condition for the I2C bus based on the passed parameters.
* @param [in]	  :   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	  :   StopCondition: Choose the type of the Stop condition must be set based on @ref I2C_StopCondition.
* @return_value :   none.
======================================================================================================================
*/
void MCAL_I2C_GenerateStop(I2C_Typedef* I2Cx, eI2C_StopCondition StopCondition);

/*
======================================================================================================================
* @Func_name    : MCAL_I2C_SendSlaveAddress
* @brief		    : Function to send the slave address on the I2C SDA line.
* @param [in]	  : I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	  : SlaveAddress: The slave address to be sent.
* @return_value : none.
* Note			    : none.
======================================================================================================================
*/
void MCAL_I2C_SendSlaveAddress(I2C_Typedef* I2Cx, uint16 SlaveAddress, eI2C_Direction direction);

/*
======================================================================================================================
* @Func_name	  : MCAL_I2C_AcknowledgeCFG
* @brief		    : Enable or disable the acknowledgement of the data based on the parameters.
* @param [in]	  : I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	  : NewState: The new state to be set for the start condition must be a value of {Enable, Disable};
======================================================================================================================
*/
void MCAL_I2C_AcknowledgeCFG(I2C_Typedef* I2Cx, FunctionalState newState);

/*
======================================================================================================================
* @Func_name	  :   MCAL_I2C_GetEventStatus
* @brief		    :   Check if a certain event has occurred or not.
* @param [in]	  :   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	  :   event:  Specifies the event to be checked, must be a value of @ref I2C_EVENTS. 
* @return_value :   EventStatus: The status of the event, Value of @ref I2C_EVENT_STATUS 
======================================================================================================================
*/
EventStatus MCAL_I2C_GetEventStatus(I2C_Typedef* I2Cx, eEvent event);

#endif /* MCAL_STM32F103X6_I2C_H_ */
