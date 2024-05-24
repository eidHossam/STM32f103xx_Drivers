/*
**************************************************************************************************************************
* file		: I2C.c
* brief     : This files contains functions to configure the I2C communication module.
* Author    : Hossam Eid
* Created on: Nov 20, 2023
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
#include "stm32f103x6_I2C.h"

/** @defgroup I2C Local Macros
  * @{
  */
  
/**
  * @}
  */

/** @defgroup I2C Local Defines 
    * @{
    */

#define I2C1_INDEX 0
#define I2C2_INDEX 1


#define I2C_CCR_SPEED_MODE_RESET    ~(1 << 15)
#define I2C_CCR_CCR_RESET           ~(0xFFF << 0)
#define I2C_TRISE_RESET             ~(0x3F << 0)
/**
  * @}
  */



/** @defgroup I2C Local Variables
  * @{
  */

I2C_CFG_t I2C_CFG_local[2] = {0, 0};
/**
  * @}
  */


/** @defgroup I2C Local Functions
  * @{
  */

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_GPIO_Set_Pins
* @brief		:   Configure the GPIO pins for I2C communication.
* @param [in]	:   I2Cx: specifies the I2C instance to be initialized can be (I2C1, I2C2). 
* @return_value :   none.
* Note			:   AF pins remapping is not supported.
======================================================================================================================
*/
static void MCAL_I2C_GPIO_Set_Pins(I2C_Typedef* I2Cx)
{
    uint8 I2Cx_SCL_Pin, I2Cx_SDA_Pin;
    GPIO_Pin_Config_t GPIO_cfg;
    
    if(I2Cx == I2C1)
    {
        /** @defgroup I2C1 Pins configuration
          * @{
          *     I2C1_SCL:   PB6
          *     I2C1_SDA:   PB7
          * @}
        */

        I2Cx_SCL_Pin = GPIO_PIN6;
        I2Cx_SDA_Pin = GPIO_PIN7;
    }else if(I2Cx == I2C2)
    {
        /** @defgroup I2C2 Pins configuration
          * @{
          *     I2C2_SCL:   PB10
          *     I2C2_SDA:   PB11
          * @}
        */

        I2Cx_SCL_Pin = GPIO_PIN10;
        I2Cx_SDA_Pin = GPIO_PIN11;
    }
   
    /*Configure the SCL pin to be AF output open-drain*/
    GPIO_cfg.pinNumber = I2Cx_SCL_Pin;
    GPIO_cfg.pinMode = GPIO_MODE_AF_OUTPUT_OD_10MHZ;

    MCAL_GPIO_Init(GPIOB, &GPIO_cfg);

    /*Configure the SDA pin to be AF output open-drain*/
    GPIO_cfg.pinNumber = I2Cx_SDA_Pin;
    GPIO_cfg.pinMode = GPIO_MODE_AF_OUTPUT_OD_10MHZ;

    MCAL_GPIO_Init(GPIOB, &GPIO_cfg);
}

/**
  * @}
  */



/**************************************************************************************************************************
*===============================================
*  			APIs functions definitions
*===============================================
*/

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_Init
* @brief		:   Intitialization of the specified I2C instance using the specified parameters in the config structure.
* @param [in]	:   I2Cx: specifies the I2C instance to be initialized can be (I2C1, I2C2).
* @param [in]	:   I2C_CFG: specifies the configuration parameters for the specified I2C instance.
* @return_value :   none.
======================================================================================================================
*/
void MCAL_I2C_Init(I2C_Typedef* I2Cx, I2C_CFG_t* I2C_CFG)
{
    /*Enable the clock and save the config structure for later use*/
    if(I2Cx == I2C1)
    {
        APB1_PERI_CLOCK_EN(APB1_I2C1);
        I2C_CFG_local[I2C1_INDEX] = *I2C_CFG;
    }else if(I2Cx == I2C2)
    {
        APB1_PERI_CLOCK_EN(APB1_I2C2);
        I2C_CFG_local[I2C2_INDEX] = *I2C_CFG;
    }
   
    uint32 tempReg = 0;

/*---------------------------- I2Cx CR2 Configuration ------------------------*/
    tempReg = I2Cx->CR2;
    uint32 FCLK1 = MCAL_RCC_GET_PCLK1();
    tempReg &= ~(I2C_CR2_FREQ_Msk);     /*Clear the FREQ bits*/
    tempReg |= (uint8)(FCLK1 / 1000000);

    /*Enable or Disable the interrupts based on the mode
      Note: Master mode only support polling mechanism, and slave mode only support
      interrupt mechanism
    */
    tempReg &= ~(I2C_MODE_SLAVE);       /*Clear all the interrupts*/
    tempReg |= I2C_CFG->I2C_Mode;

    I2Cx->CR2 = tempReg;

/*---------------------------- I2Cx CCR Configuration ------------------------*/
    tempReg = I2Cx->CCR;
    tempReg &= I2C_CCR_SPEED_MODE_RESET;    /*The only supported mode is the standard mode*/

    uint16 CCR = (uint16)(FCLK1 / (I2C_CFG->I2C_ClockSpeed << 1)); 
    tempReg &= I2C_CCR_CCR_RESET;           /*Clear the CCR bits*/
    tempReg |= CCR;

    I2Cx->CCR = tempReg;
/*---------------------------- I2Cx TRISE Configuration ------------------------*/
    tempReg = I2Cx->TRISE;
    tempReg &= I2C_TRISE_RESET;
    
    /*Maximum allowed rise time is CR2.FREQ + 1*/
    tempReg |= ((FCLK1 / 1000000) + 1);
    I2Cx->TRISE = tempReg;

/*---------------------------- I2Cx CR1 Configuration ------------------------*/
    tempReg = 0;

    tempReg |= (I2C_CFG->I2C_Ack | I2C_CFG->Slave_ClockStretchEN | I2C_CFG->Slave_GeneralCallEN);
    I2Cx->CR1 = tempReg;

/*---------------------------- I2Cx OAR1 OAR2, Interrupts Configuration ------------------------*/

    if(I2C_CFG->I2C_Mode == (I2C_MODE_SLAVE))
    {
        /*Enable the interrupts in case of slave mode*/
        if(I2Cx == I2C1)
        {
            MCAL_NVIC_EnableIRQ(NVIC_I2C1_EV_IVT_INDEX);
            MCAL_NVIC_EnableIRQ(NVIC_I2C1_ER_IVT_INDEX);
        }else if(I2Cx == I2C2)
        {
            MCAL_NVIC_EnableIRQ(NVIC_I2C2_EV_IVT_INDEX);
            MCAL_NVIC_EnableIRQ(NVIC_I2C2_ER_IVT_INDEX);
        }
        
        tempReg = 0;
        
        tempReg |= I2C_CFG->I2C1_SLAVEADD_CFG.I2C_SlaveAddressLength;
        if(I2C_CFG->I2C1_SLAVEADD_CFG.I2C_SlaveAddressLength == I2C_SLAVEADDRESS_7BIT)
        {
            tempReg |= (I2C_CFG->I2C1_SLAVEADD_CFG.I2C_PrimarySlaveAddress << 1);
        }else{
            tempReg |= (I2C_CFG->I2C1_SLAVEADD_CFG.I2C_PrimarySlaveAddress);
        }
        I2Cx->OAR1 = tempReg;

        tempReg = 0;
        tempReg |= I2C_CFG->I2C1_SLAVEADD_CFG.I2C_SlaveDualAddressEn;    

        if(I2C_CFG->I2C1_SLAVEADD_CFG.I2C_SlaveDualAddressEn == I2C_ENDUAL_ENABLE)
        {
            tempReg |= (I2C_CFG->I2C1_SLAVEADD_CFG.I2C_SecSlaveAddress << 1);    
        }

        I2Cx->OAR2 = tempReg;
    }

    MCAL_I2C_GPIO_Set_Pins(I2Cx);
    
    /*Enable the I2C instance*/
    I2Cx->CR1 |= I2C_CR1_PE;
}   

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_Reset
* @brief		:   Reset the specified UART instance to its original state.
* @param [in]	:   USART: specifies the UART instance to be reset, can be (USART1, USART2 or USART3).
* @param [out]	:   none.
* @return_value :   none.
* Note			:   none.
======================================================================================================================
*/
void MCAL_I2C_Reset(I2C_Typedef* I2Cx)
{
    if(I2Cx == I2C1)
    {
        APB1_PERI_RESET(APB1_I2C1);
        MCAL_NVIC_DisableIRQ(NVIC_I2C1_ER_IVT_INDEX);
        MCAL_NVIC_DisableIRQ(NVIC_I2C1_EV_IVT_INDEX);

    }else if(I2Cx == I2C2)
    {
        APB1_PERI_RESET(APB1_I2C2);
        MCAL_NVIC_DisableIRQ(NVIC_I2C2_ER_IVT_INDEX);
        MCAL_NVIC_DisableIRQ(NVIC_I2C2_EV_IVT_INDEX);
    }
}

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_MasterTX
* @brief		:   Function to send data from a master to a slave on the I2C bus.
* @param [in]	:   I2Cx: specifies the I2C instance to be initialized can be (I2C1, I2C2).
* @param [in]	:   I2C_MasterTxRx_CFG_t: A struct to configure the transmission options for the master.
* @note         :   This function works with a polling mechanism so if any error occours during transmission
*               :   you will be  stuck in this function, a timeout, interrupt mechanism is to be added.
======================================================================================================================
*/
void MCAL_I2C_MasterTX(I2C_Typedef* I2Cx, I2C_MasterTxRx_CFG_t* masterTxRx_cfg)
{
    uint8 i = 0;
    /*1 - Generate the start condition*/
    MCAL_I2C_GenerateStart(I2Cx, masterTxRx_cfg->startCondition, Enable);

    /*2 - Wait for EV5 to be set*/
    while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV5));

    /*3 - Send the slave address paired with the Transmission bit*/
    MCAL_I2C_SendSlaveAddress(I2Cx, masterTxRx_cfg->slaveAddress, I2C_DirectionTransmission);

    /*4 - Wait for EV6 ADDR=1, cleared by reading SR1 register followed by reading SR2.*/
    while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV6));

    /*5 - Make sure we are in the right operating mode (Master transmit)*/
    while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_MasterTransmitting));

    /*6 - Start sending the data every time EV8 or EV8_1 occurs*/
    for(i = 0; i < masterTxRx_cfg->bufferSize; i++)
    {
        /*Wait for the data register to be empty*/
        while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV8));

        I2Cx->DR = masterTxRx_cfg->TX_dataBuffer[i];
    }

    /*7 - EV8_2: TxE=1, BTF = 1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition*/
    while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV8_2));


    /*Check if the stop condition generation is enabled or disabled*/
    if(masterTxRx_cfg->stopCondition == StopConditionEN)
    {
        MCAL_I2C_GenerateStop(I2Cx,  StopConditionEN);
    }
}

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_MasterRX
* @brief		:   Function to receive data from a slave as the master on the I2C bus.
* @param [in]	:   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	:   masterTxRx_cfg: A struct to configure the transmission options for the master.
* @note         :   This functions works with a polling mechanism, a timeout, interrupt mechanism is to be added.
======================================================================================================================
*/
void MCAL_I2C_MasterRX(I2C_Typedef* I2Cx, I2C_MasterTxRx_CFG_t* masterTxRx_cfg)
{
    uint8 i = 0;
    uint8 index =  I2Cx == I2C1 ? I2C1_INDEX: I2C2_INDEX ;
    /*1 - Generate the start condition*/
    MCAL_I2C_GenerateStart(I2Cx, masterTxRx_cfg->startCondition, Enable);

    /*2 - Wait for EV5 to be set*/
    while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV5));

    /*3 - Send the slave address paired with the Reception bit*/
    MCAL_I2C_SendSlaveAddress(I2Cx, masterTxRx_cfg->slaveAddress, I2C_DirectionReception);

    /*4 - Wait for EV6 ADDR=1, cleared by reading SR1 register followed by reading SR2.*/
    while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV6));

    for(i = 0; i < masterTxRx_cfg->bufferSize; i++)
    {
        /**
         * 1) To generate the nonacknowledge pulse after the last received data byte, the ACK bit
         *    must be cleared just after reading the second last data byte (after second last RxNE
         *    event).
         * 
         * 2) To generate the Stop/Restart condition, software must set the STOP/START bit just
         *    after reading the second last data byte (after the second last RxNE event).
         */

        if(i == (masterTxRx_cfg->bufferSize - 1))
        {
            MCAL_I2C_AcknowledgeCFG(I2Cx, Disable);

            /*Check if the stop condition generation is enabled or disabled*/
            if(masterTxRx_cfg->stopCondition == StopConditionEN)
            {
                MCAL_I2C_GenerateStop(I2Cx,  StopConditionEN);
            }
        }

        /*5 - EV7: RxNe = 1, cleared by reading the DR register*/
        while(!MCAL_I2C_GetEventStatus(I2Cx, I2C_EV7));

        masterTxRx_cfg->TX_dataBuffer[i] = I2Cx->DR;
    }

    /*re-enable ACKing*/
	if(I2C_CFG_local[index].I2C_Ack == I2C_ACK_ENABLE)
	{
		MCAL_I2C_AcknowledgeCFG(I2Cx,Enable);
	}
}

/*
======================================================================================================================
* @Func_name	: MCAL_I2C_SlaveTx
* @brief	    : This function sends data on the I2C bus as a slave when called by the master
* @param [in]	: I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [out]	: packet: data to be sent.
======================================================================================================================
*/
void MCAL_I2C_SlaveTx(I2C_Typedef* I2Cx, uint8 packet)
{
    I2Cx->DR = packet;
}

/*
======================================================================================================================
* @Func_name	: MCAL_I2C_SlaveRx
* @brief		: This function receives data from the I2C bus as a slave when called by the master
* @param [in]	: I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @return_value : the received data.
======================================================================================================================
*/
uint8 MCAL_I2C_SlaveRx(I2C_Typedef* I2Cx)
{
    return I2Cx->DR;
}

/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_GenerateStart
* @brief		:   Function to generate the start condition for the I2C bus based on the passed parameters.
* @param [in]	:   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	:   startCondition: Choose the type of the start condition must be set based on @ref I2C_StartCondition.
* @param [in]	:   NewState: The new state to be set for the start condition must be a value of {Enable, Disable};
* @return_value :   none.
======================================================================================================================
*/
void MCAL_I2C_GenerateStart(I2C_Typedef* I2Cx, eI2C_StartCondition startCondition, FunctionalState NewState)
{
    /*Sense if the bus is busy or not in case of normal start condition*/
    if(startCondition != RepeatedStartCondition)
    {
        while(MCAL_I2C_GetEventStatus(I2Cx, I2C_LineBusy));
    }

    /*
        Bit 8 START: Start generation
                This bit is set and cleared by software and cleared by hardware when start is sent or PE=0.
                In Master Mode:
                0: No Start generation
                1: Repeated start generation
                In Slave mode:
                0: No Start generation
                1: Start generation when the bus is free
    */

    if(NewState == Enable)
    {
        /*Enable the start condition generation*/
        I2Cx->CR1 |= I2C_CR1_START;
    }else{
        /*Disable the start condition generation*/
        I2Cx->CR1 &= ~(I2C_CR1_START);
    }

}

/*
======================================================================================================================
* @Func_name	  :   MCAL_I2C_GenerateStop
* @brief		  :   Function to generate the Stop condition for the I2C bus based on the passed parameters.
* @param [in]	  :   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	  :   NewState: The new state to be set for the Stop condition must be a value of {Enable, Disable};
* @return_value   :   none.
======================================================================================================================
*/
void MCAL_I2C_GenerateStop(I2C_Typedef* I2Cx, eI2C_StopCondition StopCondition)
{
    /*Enable or Disable the stop condition generation based on the user selection*/
    I2Cx->CR1 = ((I2Cx->CR1 & ~(I2C_CR1_STOP)) | (StopCondition << I2C_CR1_STOP_Pos));
}


/*
======================================================================================================================
* @Func_name    : MCAL_I2C_SendSlaveAddress
* @brief		: Function to send the slave address on the I2C SDA line.
* @param [in]	: I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	: SlaveAddress: The slave address to be sent.
* @return_value : none.
* Note			: none.
======================================================================================================================
*/
void MCAL_I2C_SendSlaveAddress(I2C_Typedef* I2Cx, uint16 SlaveAddress, eI2C_Direction direction)
{   
    uint8 I2C_index = 0;
    uint16 address = 0;
    /*Specify which instance we are working with*/
    I2C_index = (I2Cx == I2C1)? I2C1_INDEX: I2C2_INDEX;

    if(I2C_CFG_local[I2C_index].I2C1_SLAVEADD_CFG.I2C_SlaveAddressLength == I2C_SLAVEADDRESS_7BIT)
    {   
        /*
        *   Pair the 7-bit address with the R/w Command in the LSB
        *
        *   In 7-bit addressing mode:
        *   – To enter Transmitter mode, a master sends the slave address with LSB reset.
        *   – To enter Receiver mode, a master sends the slave address with LSB set.
        */
        address = (SlaveAddress << 1);
        address = (address & (0xFE)) | (0x01 & direction);
        
        I2Cx->DR = address;
    }else{

        /*ToDo: 10-bit slave address yet to be supported*/
    }

}

/*
======================================================================================================================
* @Func_name    : MCAL_I2C_AcknowledgeCFG
* @brief        : Enable or disable the acknowledgement of the data based on the parameters.
* @param [in]	: I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	: NewState: The new state to be set for the start condition must be a value of {Enable, Disable};
======================================================================================================================
*/
void MCAL_I2C_AcknowledgeCFG(I2C_Typedef* I2Cx, FunctionalState newState)
{
    /**
     * Bit 10 ACK: Acknowledge enable
     *  This bit is set and cleared by software and cleared by hardware when PE=0.
     *  0: No acknowledge returned
     *  1: Acknowledge returned after a byte is received (matched address or data)
     */

    if(newState == Enable)
    {

        I2Cx->CR1 |= I2C_CR1_ACK;
    }else
    {

        I2Cx->CR1 &= ~(I2C_CR1_ACK);
    }
}


/*
======================================================================================================================
* @Func_name	:   MCAL_I2C_GetEventStatus
* @brief		:   Check if a certain event has occurred or not.
* @param [in]	:   I2Cx: specifies the I2C instance to be used, can be (I2C1, I2C2).
* @param [in]	:   event:  Specifies the event to be checked, must be a value of @ref I2C_EVENTS. 
* @return_value :   EventStatus: The status of the event, Value of @ref I2C_EVENT_STATUS 
======================================================================================================================
*/
EventStatus MCAL_I2C_GetEventStatus(I2C_Typedef* I2Cx, eEvent event)
{
    uint8 EventStatus;
    uint16 dummyRead;
    switch (event)
    {

    case I2C_LineBusy:
        /*
         *   Bit 1 BUSY: Bus busy
         *       0: No communication on the bus
         *       1: Communication ongoing on the bus
         *       – Set by hardware on detection of SDA or SCL low
         *       – cleared by hardware on detection of a Stop condition.
         *       It indicates a communication in progress on the bus. This information is still updated when
         *       the interface is disabled (PE=0).
        */
        if(I2Cx->SR2 & (I2C_SR2_BUSY_Msk))
            EventStatus =  Set;
        else
            EventStatus =  Reset;
        break;

    case I2C_EV5:
        /*
        *   EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
        *
        *   Bit 0 SB: Start bit (Master mode)
        *   0: No Start condition
        *   1: Start condition generated.
        *   – Set when a Start condition generated.
        *   – Cleared by software by reading the SR1 register followed by writing the DR register, or by
        *   hardware when PE=0
        */

        if(I2Cx->SR1 & (I2C_SR1_SB))
        {
            EventStatus = Set;
        }else{
            EventStatus = Reset;
        }
        break;


    case I2C_EV6:
        /*
         *  EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
         *
         * Bit 1 ADDR: Address sent (master mode)/matched (slave mode)
         *      Address matched (Slave)
         *       0: Address mismatched or not received.
         *       1: Received address matched.
         * 
         *     Address sent (Master)
         *       0: No end of address transmission
         *       1: End of address transmission
         * 
         *      Note: ADDR is not set after a NACK reception 
         * */

        if(I2Cx->SR1 & (I2C_SR1_ADDR))
        {
            EventStatus = Set;
        }else{
            EventStatus = Reset;
        }

        dummyRead = I2Cx->SR2;
        break;

    case I2C_EV7:
        /**
         * EV7: RxNe = 1, cleared by reading the DR register
         * 
         * Bit 6 RxNE: Data register not empty (receivers)
         *  0: Data register empty
         *  1: Data register not empty
         */

        if(I2Cx->SR1 & (I2C_SR1_RXNE))
        {
            EventStatus = Set;
        }else{
            EventStatus = Reset;
        }

        break;
    case I2C_MasterTransmitting:
        /**
         * Check if the bus is busy and we are in Master transmission mode
         * 
         *  Bit 0 MSL: Master/slave
         *  0: Slave Mode
         *  1: Master Mode
         * 
         *  Bit 1 BUSY: Bus busy
         *  0: No communication on the bus
         *  1: Communication ongoing on the bus
         * 
         *  Bit 2 TRA: Transmitter/receiver
         *  0: Data bytes received
         *  1: Data bytes transmitted
         */

        if((I2Cx->SR2 & (I2C_MasterTransmitting)) == (I2C_MasterTransmitting))
        {
            EventStatus = Set;
        }else{
            EventStatus = Reset;
        }
        break;

    case I2C_EV8:
    case I2C_EV8_1:
        /**
         *  EV8_1: TxE=1, shift register empty, data register empty, write Data1 in DR.
         *  EV8: TxE=1, shift register not empty, .data register empty, cleared by writing DR register
         * 
         *  Bit 7 TxE: Data register empty (transmitters)
         *  0: Data register not empty
         *  1: Data register empty
         * 
         *  TxE is not set if a NACK is received,
         */
        if(I2Cx->SR1 & (I2C_SR1_TXE))
        {
            EventStatus = Set;
        }else{
            EventStatus = Reset;
        }
        break;

    case I2C_EV8_2:
        /**
         * EV8_2: TxE=1, BTF = 1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition
         * 
         * Bit 2 BTF: Byte transfer finished
         *  0: Data byte transfer not done
         *  1: Data byte transfer succeeded
         * 
         * Bit 7 TxE: Data register empty (transmitters)
         *  0: Data register not empty
         *  1: Data register empty
         */
        if((I2Cx->SR1 & (I2C_EV8_2)) == (I2C_EV8_2))
        {
            EventStatus = Set;
        }else{
            EventStatus = Reset;
        }
        break;

    default:
        break;
    }

    return EventStatus;
}

void Slave_States (I2C_Typedef* I2Cx ,eSlave_State state)
{
	uint8 index =    I2Cx == I2C1 ? I2C1_INDEX: I2C2_INDEX ;

	switch (state)
	{

	case I2C_ERROR_AF:
	{
		/*make sure that the slave is really in transmitter mode*/
		if(I2Cx->SR2 & ( I2C_SR2_TRA))
		{
			/*Slave Shouldn't Send anything else*/
		}
		break ;
	}

	case I2C_EV_STOP:
	{
		/*make sure that the slave is really in transmitter mode*/
		if(I2Cx->SR2 & ( I2C_SR2_TRA))
		{
			/*Notify APP that the Stop Condition is sent by the master*/
			I2C_CFG_local[index].p_I2C_ISR_callback(I2C_EV_STOP);
		}

		break ;
	}

	case I2C_EV_ADDR_Matched:
	{
		/*Notify APP that the Stop Condition is sent by the master*/
		I2C_CFG_local[index].p_I2C_ISR_callback(I2C_EV_ADDR_Matched) ;

		break ;
	}

	case I2C_EV_DATA_REQ:
	{
		/*make sure that the slave is really in transmitter mode*/
		if(I2Cx->SR2 & ( I2C_SR2_TRA))
		{
			/*the APP layer should send the data (MCAL_I2C_SlaveSendData ) in this state*/
			I2C_CFG_local[index].p_I2C_ISR_callback(I2C_EV_DATA_REQ) ;
		}

		break ;
	}
	case I2C_EV_DATA_RCV:
	{
		/*make sure that the slave is really in receiver mode*/
		if(!(I2Cx->SR2 & ( I2C_SR2_TRA)))
		{
			/*the APP layer should read the data (MCAL_I2C_SlaveReceiveData ) in this state*/
			I2C_CFG_local[index].p_I2C_ISR_callback(I2C_EV_DATA_RCV) ;
		}

		break ;
	}
	}

}

void I2C_ISR(I2C_Typedef* I2Cx)
{
    volatile uint32 dummy_read = 0 ;

    /*Interrupt handling for both master and slave mode of a device*/
	uint32 temp1, temp2, temp3;

	temp1 = I2Cx->CR2 & (I2C_CR2_ITEVTEN);
	temp2 = I2Cx->CR2 & (I2C_CR2_ITBUFEN);
	temp3 = I2Cx->SR1 & (I2C_SR1_STOPF);


	/* Handle For interrupt generated by STOPF event
	    Note : Stop detection flag is applicable only slave mode*/
	if(temp1 && temp3)
	{
		/*  STOP flag is set
		    Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )
        */
		I2Cx->CR1 |= 0x0000;
		Slave_States(I2Cx,I2C_EV_STOP);
	}
	
    /*----------------------------------------------------------*/
	
    temp3  = I2Cx->SR1 & (I2C_SR1_ADDR);

	/*  Handle For interrupt generated by ADDR event
	    Note : When master mode : Address is sent
		When Slave mode   : Address matched with own address
    */

	if(temp1 && temp3)
	{
		/*  interrupt is generated because of ADDR event
		    check for device mode
        */
		if(I2Cx->SR2 & ( I2C_SR2_MSL))
		{
			/*master*/
		}else
		{
			/*  slave mode
			    clear the ADDR flag ( read SR1 , read SR2)
            */
			dummy_read = I2Cx->SR1;
			dummy_read = I2Cx->SR2;
			Slave_States(I2Cx,I2C_EV_ADDR_Matched);
		}
	}
	
    /*----------------------------------------------------------*/

	temp3  = I2Cx->SR1 & ( I2C_SR1_TXE);
	
    /* Handle For interrupt generated by TXE event*/
	if(temp1 && temp2 && temp3)
	{
		/*Check for device mode*/
		if(I2Cx->SR2 & (I2C_SR2_MSL))
		{
		}else
		{
			/*slave*/
			Slave_States(I2Cx,I2C_EV_DATA_REQ);
		}
	}

    /*----------------------------------------------------------*/

	temp3  = I2Cx->SR1 & ( I2C_SR1_RXNE);
	
    /*  Handle For interrupt generated by SB=1, cleared by reading SR1 register 
        followed by writing DR register with  Address.
    */
	if(temp1 && temp2 && temp3)
	{
		/*check device mode .*/
		if(I2Cx->SR2 & ( I2C_SR2_MSL))
		{
			/*The device is master*/
		}else
		{
			/*slave*/
			Slave_States(I2Cx,I2C_EV_DATA_RCV);
		}
	}    
}

void I2C1_EV_IRQHandler(void)
{
    I2C_ISR(I2C1);
}

void I2C2_EV_IRQHandler(void)
{
    I2C_ISR(I2C2);
}
