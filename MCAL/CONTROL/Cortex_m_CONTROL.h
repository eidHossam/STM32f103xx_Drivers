/**
 * @file Cortex_m_CONTROL.h
 * 
 * @brief this file declares the APIs used to control the CPU.
 * 
 * @author Hossam_Eid (eidhossam7@gmail.com)
 * 
 * @date 25-03-2024
 * 
 * @version 1.0
 */

#ifndef MCAL_CONTROL_CORTEX_M_CONTROL_H_
#define MCAL_CONTROL_CORTEX_M_CONTROL_H_

/** @defgroup  INCLUDES
  * @{
  */
#include "Platform_Types.h"
#include "Bit_Math.h"
/**
  * @}
  */

/**************************************************************************************************************************
*===============================================
* User type definitions
*===============================================
*/

/**
 * @brief This enum holds the possible values for the CPU access levels.
 * 
 */
typedef enum {
    privilegedAccess,
    unprivilegedAccess
}eCPU_AccessLevel_t;

/**************************************************************************************************************************
================================================
*               APIs Supported
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
eStatus_t CPU_changeAccessLevel(eCPU_AccessLevel_t copy_accessLevel);


#endif /* MCAL_CONTROL_CORTEX_M_CONTROL_H_ */
