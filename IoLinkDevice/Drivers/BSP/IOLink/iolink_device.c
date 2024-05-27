/**
 ******************************************************************************
 * @file    iolink_device.c
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    January 8th, 2018
 * @brief   This file provides common driver functions for IO-Link devices
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "iolink_device.h"

/** @addtogroup BSP
 * @{
 */

/** @defgroup IOLINK_DEVICE IOLINK_DEVICE
 * @{
 */

/** @defgroup IOLINK_DEVICE_Private_Types_Definitions IOLINK_DEVICE Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup IOLINK_DEVICE_Private_Defines IOLINK_DEVICE Private Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup IOLINK_DEVICE_Private_Constants IOLINK_DEVICE Private Constants
 * @{
 */

/**
 * @}
 */

/** @defgroup IOLINK_DEVICE_Private_Macros IOLINK_DEVICE Private Macros
 * @{
 */
/** Error when trying to call undefined functions via gpIOLinkDeviceHandle */
#define IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(errorNb)   (BSP_IOLinkDevice_ErrorHandler(IOLINK_DEVICE_ERROR_TAG|(errorNb)))   

/**
 * @}
 */   
   
/** @defgroup IOLINK_DEVICE_Private_Variables IOLINK_DEVICE Private Variables
 * @{
 */

static iolinkDeviceDrv_t *gpIOLinkDeviceHandle = (void *)0;
static uint16_t gIOLinkDeviceBoardId;

/**
 * @}
 */

/** @defgroup IOLINK_DEVICE_Weak_Private_Functions IOLINK_DEVICE Weak Private Functions
 * @{
 */
/** Get IOLink device handle for L6362A */
__weak iolinkDeviceDrv_t* L6362a_GetIOLinkDeviceHandle(void){return ((iolinkDeviceDrv_t *)(void *)0);}

/**
 * @}
 */

/** @defgroup BSP_IOLinkDevice_Functions BSP IOLINK_DEVICE Functions
  * @{
  */   

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_AttachErrorHandler(void (*callback)(uint16_t))
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->AttachErrorHandler != (void *)0))
  {
    gpIOLinkDeviceHandle->AttachErrorHandler(callback);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(0U);
  }
}

/******************************************************//**
 * @brief  Attaches a user callback to the Fault interrupt Handler.
 * The call back will be then called each time the library 
 * detects a FAULT signal falling or rising  edge.
 * @param[in] callback Name of the callback to attach 
 * to the Fault interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_AttachFaultInterrupt(void (*callback)(uint8_t))
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->AttachFaultInterrupt != 0))
  {
    gpIOLinkDeviceHandle->AttachFaultInterrupt(callback);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(1U);
  }  
}

/******************************************************//**
 * @brief  Attaches a user callback to the Overload interrupt Handler.
 * The call back will be then called each time the library 
 * detects a OL signal falling or rising  edge.
 * @param[in] callback Name of the callback to attach 
 * to the Fault interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_AttachOLInterrupt(void (*callback)(uint8_t))
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->AttachOLInterrupt != 0))
  {
    gpIOLinkDeviceHandle->AttachOLInterrupt(callback);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(2U);
  }  
}


/******************************************************//**
 * @brief  Attaches a user callback to the Out IQ received
 * data interrupt
 * @param[in] callback Name of the callback to attach 
 * to the OUT IQ received data interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_AttachOutIqDataHandler(void (*callback)(uint8_t, uint8_t))
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->AttachOutIqDataHandler != 0))
  {
    gpIOLinkDeviceHandle->AttachOutIqDataHandler(callback);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(3U);
  }  
}

/******************************************************//**
 * @brief IO LInk Device control error handler
 * @param[in] error number of the error
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_ErrorHandler(uint16_t error)
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->ErrorHandler != 0))
  {
    gpIOLinkDeviceHandle->ErrorHandler(error);
  }  
  else
  {
    while(TRUE)
    {
      /* Infinite loop as Error handler must be defined*/
    }
  }
}


/******************************************************//**
 * @brief  Handlers of the fault interrupt which calls the user callback (if defined)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_FaultInterruptHandler(uint8_t deviceId)
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->FaultInterruptHandler != 0))
  {
    gpIOLinkDeviceHandle->FaultInterruptHandler(deviceId);
  }    
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(4U);
  }  
}

/******************************************************//**
 * @brief  Handlers of the OL interrupt which calls the user callback (if defined)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_OLInterruptHandler(uint8_t deviceId)
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->OLInterruptHandler != 0))
  {
    gpIOLinkDeviceHandle->OLInterruptHandler(deviceId);
  }    
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(5U);
  }  
}


/******************************************************//**
 * @brief  Handlers of the OUT IQ received data interrupt 
 *  which calls the user callback (if defined)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] data Received byte
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_OutIqDataHandler(uint8_t deviceId, uint8_t data)
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->OutIqDataHandler != 0))
  {
    gpIOLinkDeviceHandle->OutIqDataHandler(deviceId, data);
  }    
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(6U);
  }    
}

/******************************************************//**
 * @brief Get board Id  
 * @retval IO Link Device board Id
 **********************************************************/
uint16_t BSP_IOLinkDevice_GetBoardId(void)
{
  return (gIOLinkDeviceBoardId);
}

/******************************************************//**
 * @brief Get IO-Link Com Mode 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval comMode COM1,COM2 or COM3
 **********************************************************/
uint8_t BSP_IOLinkDevice_GetComMode(uint8_t deviceId)
{
  uint8_t value = 0U;
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->GetComMode != 0))
  {
    value = (uint8_t)gpIOLinkDeviceHandle->GetComMode(deviceId);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(7U);
  }
  return (value);
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval BSP_IOLinkDevice_FW_VERSION
 * @note the format is (MAJOR_VERSION<<16)|(MINOR_VERSION<<8)|(PATCH_VERSION)
 * with major, minor and patch versions coded on 8 bits. 
 **********************************************************/
uint32_t BSP_IOLinkDevice_GetFwVersion(void)
{
  uint32_t version = 0U;
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->GetFwVersion != 0))
  {
    version = gpIOLinkDeviceHandle->GetFwVersion();
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(8U);
  }  
  return(version);
}

/******************************************************//**
 * @brief Return the number of devices in the daisy chain 
 * @retval number of devices from 0 to MAX_NUMBER_OF_DEVICES - 1
 **********************************************************/
uint8_t BSP_IOLinkDevice_GetNbDevices(void)
{
  uint8_t value = 0U;
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->GetNbDevices != 0))
  {
    value = gpIOLinkDeviceHandle->GetNbDevices();
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(9U);
  }
  return (value);
}

/******************************************************//**
 * @brief Get the status of the enable pin
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval Status of the Output enable
 **********************************************************/
uint8_t BSP_IOLinkDevice_GetEnable(uint8_t deviceId)
{
  uint8_t status = 0U;
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->GetEnable != 0))
  {
    status = gpIOLinkDeviceHandle->GetEnable(deviceId);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(10U);
  }
  return (status);
}

/******************************************************//**
 * @brief Get the Fault state
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval state 0: no fault, 1: fault repoted by Diag pin 
 *               2: fault repoted by OL pin 
 *               3: fault repoted by both Diag and OL pin 
 **********************************************************/
uint8_t BSP_IOLinkDevice_GetFaultState(uint8_t deviceId)
{
  uint8_t status = 0U;
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->GetFaultState != 0))
  {
    status = gpIOLinkDeviceHandle->GetFaultState(deviceId);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(11U);
  }
  return (status);
}

/******************************************************//**
 * @brief Initialises the IO Link Device driver. 
 * This function has to be called one time for each device. 
 * The number of devices is incremented in the driver up to the maximum 
 * allowed. Calling this function a number of times greater than the
 * maximum number triggers an error in the driver.
 * @param[in] id Component Id (6362, ,...)
 * @param[in] initDeviceParameters Initialization structure for one device
 * @retval None
 **********************************************************/
void BSP_IOLinkDevice_Init(uint16_t id, void* initDeviceParameters)
{
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->Init != 0))
  {
    gpIOLinkDeviceHandle->Init(initDeviceParameters);
  }  
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(13U);
  }  
}

/******************************************************//**
 * @brief Select IO-Link Com Mode 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] comMode COM1,COM2 or COM3
 * @retval TRUE if successfull, FALSE if failure
 **********************************************************/
bool BSP_IOLinkDevice_SelectComMode(uint8_t deviceId, iolinkComMode_t comMode)
{
  bool status = FALSE; 
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->SelectComMode != 0))
  {
    status = gpIOLinkDeviceHandle->SelectComMode(deviceId, comMode);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(14U);
  }
  return (status);
}

/******************************************************//**
 * @brief Send data to In C/Q of specified port
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] data byte to send
 * @retval TRUE if successfull, FALSE if failure
 **********************************************************/
bool BSP_IOLinkDevice_SendInData(uint8_t deviceId, uint8_t data)
{
  bool status = FALSE; 
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->SendInData != 0))
  {
    status = gpIOLinkDeviceHandle->SendInData(deviceId, data);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(15U);
  }
  return (status);
}

/******************************************************//**
 * @brief Set the number of devices in the daisy chain
 * @param[in] id Component Id (6362,...)
 * @param[in] nbDevices the number of devices to be used 
 * from 0 to MAX_NUMBER_OF_DEVICES - 1
 * @retval TRUE if successfull, FALSE if failure, attempt 
 * to set a number of devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool BSP_IOLinkDevice_SetNbDevices(uint16_t id, uint8_t nbDevices)
{
  gIOLinkDeviceBoardId = id;
  bool status = FALSE;
  if (id == BSP_IOLINK_DEVICE_BOARD_ID_L6362A)
  {
    gpIOLinkDeviceHandle = L6362a_GetIOLinkDeviceHandle();
  }
  else
  {
    gpIOLinkDeviceHandle = (void *)0;
  }
  if ((gpIOLinkDeviceHandle != (void *)0)&&
      (gpIOLinkDeviceHandle->SetNbDevices != 0U)&&
      (nbDevices != 0U))
  {
    status = gpIOLinkDeviceHandle->SetNbDevices(nbDevices);
  }
  return (status);
}

/******************************************************//**
 * @brief Set the enable pin
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] enable 1 to enable, 0 to disable
 * @retval TRUE if successfull, FALSE if failure
 **********************************************************/
bool BSP_IOLinkDevice_SetEnable(uint8_t deviceId, uint8_t enable)
{
  bool status = FALSE; 
  
  if ((gpIOLinkDeviceHandle != (void *)0)&&(gpIOLinkDeviceHandle->SetEnable != 0))
  {
    status = gpIOLinkDeviceHandle->SetEnable(deviceId, enable);
  }
  else
  {
    IOLINK_DEVICE_ERROR_UNDEFINED_FUNCTION(16U);
  }
  return (status);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
