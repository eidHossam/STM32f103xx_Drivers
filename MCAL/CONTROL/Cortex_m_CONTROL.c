/**
 * @file Cortex_m_CONTROL.c
 * 
 * @brief this file implements the APIs used to control the CPU.
 * 
 * @author Hossam_Eid (eidhossam7@gmail.com)
 * 
 * @date 25-03-2024
 * 
 * @version 1.0
 */

/** @defgroup INCLUDES
  * @{
  */
#include "Cortex_m_CONTROL.h"
/**
  * @}
  */

/** @defgroup LOCAL_MACROS
  * @{
  */
#define THREAD_MODE_      0
#define HANDLER_MODE_     1
/**
  * @}
  */

/**************************************************************************************************************************
===============================================
*  				APIs Definition
*===============================================
*/

/**
 * @brief This function changes the the access level of the CPU.
 * 
 * @param copy_accessLevel: new access level of the CPU must be a value of:
 * privilegedAccess: change access level of the CPU to privileged to be able to access all resources.
 * unprivilegedAccess: Limit the user access to resources.
 * 
 * @return eStatus_t : status of the operation.
 * E_OK: the operation was successful.
 * E_NOK: the operation was not unsuccessful.
 */
eStatus_t CPU_changeAccessLevel(eCPU_AccessLevel_t copy_accessLevel)
{
    eStatus_t LOC_eStatus = E_OK;
    eCPU_AccessLevel_t LOC_eCPUAccessLevel;
    uint32 LOC_u32CntrlRegValue;
    uint8 LOC_u8CurrentCPUMode;
    uint32 LOC_u32IPSR;

    /*Read the current value from the IPSR register*/
    __asm("MRS %[out], IPSR" :[out] "=r" (LOC_u32IPSR));

    /*
      Decide if we are in thread or handler mode based on the ISR_NUMBER bits [0 : 8]
      [8:0] ISR_NUMBER This is the number of the current exception:
            0 = Thread mode
            else = Handler mode
    */
    LOC_u8CurrentCPUMode = ((LOC_u32IPSR & 0x1FFu) == 0)? THREAD_MODE_ : HANDLER_MODE_;

    /*Read the current value from the CONTRO: register*/
    __asm("MRS %[out], CONTROL" :[out] "=r" (LOC_u32CntrlRegValue));
    
    /*
      Decide if we are in privileged or unprivileged mode based on bit 0
      [0] nPRIV Defines the Thread mode privilege level:
          0 = Privileged
          1 = Unprivileged.
    */
    LOC_eCPUAccessLevel = (READ_BIT(LOC_u32CntrlRegValue, 0) == 0)? privilegedAccess : unprivilegedAccess;

    switch (copy_accessLevel)
    {
    case privilegedAccess:
        if(LOC_u8CurrentCPUMode == HANDLER_MODE_ || LOC_eCPUAccessLevel  == privilegedAccess)
        {
          /*Clear the access level bit0*/
          __asm("MRS R0, CONTROL  \n\t"
                "LSR R0, R0, #0x1 \n\t"
                "LSL R0, R0, #0x1 \n\t"
                "MSR CONTROL, R0");
        }else{

            LOC_eStatus = E_NOK;
        }
        break;

    case unprivilegedAccess:

        /*Set the access level bit (bit 0) to one*/
        __asm("MRS R0, CONTROL  \n\t"
              "ORR R0, R0, #0x1 \n\t"
              "MSR CONTROL, R0");
        break;
    
    default:
            LOC_eStatus = E_NOK;
        break;
    }

    return LOC_eStatus;
}
