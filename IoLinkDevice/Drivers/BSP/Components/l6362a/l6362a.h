/**
  ******************************************************************************
  * @file    l6362a.h 
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 8th, 2018
  * @brief   Header for L6362a driver (IO-Link communication transceiver device IC)
  * @note    (C) COPYRIGHT 2018 STMicroelectronics
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
#ifndef L6362A_H
#define L6362A_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "l6362a_target_config.h"
#include "iolink_device.h"   
   
/** @addtogroup BSP
  * @{
  */   
   

/** @addtogroup L6362A
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup L6362A_Exported_Constants L6362A Exported Constants 
  * @{
  */   
/** Current FW major version */
#define L6362A_FW_MAJOR_VERSION (uint8_t)(1U)
/** Current FW minor version */
#define L6362A_FW_MINOR_VERSION (uint8_t)(0U)
/** Current FW patch version */
#define L6362A_FW_PATCH_VERSION (uint8_t)(0U)
/** Current FW version */
#define L6362A_FW_VERSION       (uint32_t)((L6362A_FW_MAJOR_VERSION<<16U)|\
                                          (L6362A_FW_MINOR_VERSION<<8U)|\
                                          (L6362A_FW_PATCH_VERSION))

/** The maximum number of devices connected together */
#define MAX_NUMBER_OF_DEVICES                 (1U)




  /**
  * @}
  */

/* Exported Types  -------------------------------------------------------*/

/** @defgroup L6362A_Exported_Types L6362A Exported Types
  * @{
  */   

/** @defgroup L6362A_Initialization_Structure L6362A Initialization Structure
  * @{
  */
/** IO Link Device driver initialization structure definition   */

typedef struct
{
  /**Initial  COM mode */
  iolinkComMode_t comMode; 
} l6362a_Init_t;

  

/**
  * @}
  */

/** Device Parameters Structure Type */
typedef struct {
  /** COM mode */
  iolinkComMode_t comMode;
}deviceParams_t; 

/** @defgroup Device_Parameters Device Parameters
  * @{
  */

/** Driver Parameters Structure Type */
typedef struct {
    /**device parameters  */
  deviceParams_t device[MAX_NUMBER_OF_DEVICES];
}driverParams_t; 
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/



/** @defgroup L6362A_Exported_Functions L6362A Exported Functions
  * @{
  */   

/** @defgroup L6362A_Library_Functions L6362A Library Functions
  * @{
  */   
void L6362a_AttachErrorHandler(void (*callback)(uint16_t));     /* Attach a user callback to the error handler */
void L6362a_AttachDiagInterrupt(void (*callback)(uint8_t));      /* Attach a user callback to the Diag Interrupt */
void L6362a_AttachOLInterrupt(void (*callback)(uint8_t));       /* Attach a user callback to the OL Interrupt */  
void L6362a_AttachOutIqDataHandler(void (*callback)(uint8_t, uint8_t)); /* Attach a user callback to the Out IQ received data Interrupt */
iolinkComMode_t L6362a_GetComMode(uint8_t deviceId);            /* Get IOLink com mode */
uint32_t L6362a_GetFwVersion(void);                             /* Return the FW version */
iolinkDeviceDrv_t* L6362a_GetIOLinkDeviceHandle(void);          /* Return handle of the IO Link device driver */
uint8_t L6362a_GetFaultState(uint8_t deviceId);                  /* Return Fault state (Diag and OL states)*/
uint8_t L6362a_GetEnable(uint8_t deviceId);                     /* Return the Enable pin state */
uint8_t L6362a_GetNbDevices(void);                              /* Return the number of initiliased devices */
void L6362a_Init(void* pInit);                                  /* Start the L6362A library */
uint16_t L6362a_ReadId(void);                                   /* Read Id to get driver instance */
bool L6362a_SelectComMode(uint8_t deviceId, iolinkComMode_t comMode); /* Select com mode */
bool L6362a_SendInData(uint8_t deviceId, uint8_t data);         /* SendIn data  */
bool L6362a_SetNbDevices(uint8_t nbDevices);                    /* Set the number of devices */
bool L6362a_SetEnable(uint8_t deviceId, uint8_t enable);        /* Enable/Disable the L6362A */




/**
  * @}
  */

/** @defgroup L6362a_Board_Linked_Functions L6362A Board Linked Functions
  * @{
  */   
/**Delay of the requested number of milliseconds */
extern void L6362a_Board_Delay(uint32_t delay);      
/** Get diag pin state */
extern uint8_t L6362a_Board_GetDiagPinState(uint8_t deviceId); 
/** Get enable pin state */
extern uint8_t L6362a_Board_GetEnablePinState(uint8_t deviceId);
/** Get OL (overload) pin state */
extern uint8_t L6362a_Board_GetOLPinState(uint8_t deviceId); 
/**Initialise GPIOs used for L6362As */
extern void L6362a_Board_GpioInit(uint8_t deviceId);   
/**Disable EN (enable) pin */
extern uint8_t L6362a_Board_ResetEnablePin(uint8_t deviceId);   
/**Enable EN (enable) pin */
extern uint8_t L6362a_Board_SetEnablePin(uint8_t deviceId);
/** Initialise the UART used for L6362A */
extern uint8_t L6362a_Board_UartInit(uint8_t deviceId);   
/** Set the UART baudrate used for L6362A */
extern uint8_t L6362a_Board_UartSetBaudrate(uint8_t deviceId, uint32_t baudrate);
/** Send UART data to L6362A */
extern uint8_t L6362a_Board_UartSendData(uint8_t deviceId, uint8_t data);

/**
  * @}
  */


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

#endif /* #ifndef L6362A_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
