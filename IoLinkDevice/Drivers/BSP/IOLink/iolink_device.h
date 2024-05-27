/**
  ******************************************************************************
  * @file    iolink_device.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 8th, 2018
  * @brief   This file provides common definitions for IO-Link devices
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IOLINK_DEVICE_H
#define IOLINK_DEVICE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "iolink.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup IOLINK_DEVICE
  * @{
  */

/** @defgroup IOLINK_DEVICE_Exported_Types IOLINK_DEVICE Exported Types
  * @{
  */



/**
  * @}
  */

/** @defgroup IOLINK_DEVICE_Exported_Constants IOLINK_DEVICE Exported Constants
  * @{
  */
/** IOLINK_DEVICE error tag (used when trying to call undefined functions via iolinkDeviceDrvHandle) */
#define IOLINK_DEVICE_ERROR_TAG   (0x2800U)      

/**IOLink Device board id for L6362 */
#define BSP_IOLINK_DEVICE_BOARD_ID_L6362A  (6362U)
 
/**
  * @}
  */


/** @defgroup IOLINK_DEVICE_Exported_Macros IOLINK_DEVICE Exported Macros
  * @{
  */
#if  defined ( __GNUC__ )
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
#endif /* __GNUC__ */
/**
  * @}
  */

/** @defgroup IOLINK_DEVICE_Weak_Function_Prototypes IOLINK_DEVICE Weak Function Prototypes
  * @{
  */
__weak iolinkDeviceDrv_t* L6362a_GetIOLinkDeviceHandle(void);
/**
  * @}
  */   
   
/** @defgroup IOLINK_DEVICE__Exported_Functions IOLINK_DEVICE_ Exported Functions
  * @{
  */
void BSP_IOLinkDevice_AttachErrorHandler(void (*callback)(uint16_t));
void BSP_IOLinkDevice_AttachFaultInterrupt(void (*callback)(uint8_t));
void BSP_IOLinkDevice_AttachOLInterrupt(void (*callback)(uint8_t));
void BSP_IOLinkDevice_AttachOutIqDataHandler(void (*callback)(uint8_t, uint8_t));
void BSP_IOLinkDevice_ErrorHandler(uint16_t error);
void BSP_IOLinkDevice_FaultInterruptHandler(uint8_t deviceId);
void BSP_IOLinkDevice_OLInterruptHandler(uint8_t deviceId);
void BSP_IOLinkDevice_OutIqDataHandler(uint8_t deviceId, uint8_t data);
uint16_t BSP_IOLinkDevice_GetBoardId(void);
uint8_t BSP_IOLinkDevice_GetComMode(uint8_t deviceId);
uint8_t BSP_IOLinkDevice_GetEnable(uint8_t deviceId);
uint32_t BSP_IOLinkDevice_GetFwVersion(void); 
uint8_t BSP_IOLinkDevice_GetNbDevices(void);
uint8_t BSP_IOLinkDevice_GetFaultState(uint8_t deviceId);
void BSP_IOLinkDevice_Init(uint16_t id, void* initDeviceParameters); 
bool BSP_IOLinkDevice_SelectComMode(uint8_t deviceId, iolinkComMode_t comMode);
bool BSP_IOLinkDevice_SendInData(uint8_t deviceId, uint8_t data);
bool BSP_IOLinkDevice_SetEnable(uint8_t deviceId, uint8_t enable);
bool BSP_IOLinkDevice_SetNbDevices(uint16_t id, uint8_t nbDevices);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* IOLINK_DEVICE_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
