/**
 ******************************************************************************
 * @file    iolink.h
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    January 8th, 2018
 * @brief   This header file contains the functions prototypes for IOLink master and device
 * drivers
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
#ifndef IOLINK_H
#define IOLINK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @defgroup IOLink IOLink
  * @{
  */

/** @defgroup IOLink_Exported_Constants IOLink Exported Constants
  * @{
  */   
   
/** boolean for false condition */
#ifndef FALSE
#define FALSE (0)
#endif
/** boolean for true condition */
#ifndef TRUE
#define TRUE  (1)
#endif

   /**
  * @}
  */
     
/** @defgroup IOLink_Exported_Types IOLink Exported Types
  * @{
  */

/**
 * @brief  IOLink boolean type definition
 */
typedef uint8_t  bool;

/** @defgroup IoLink_Com_Mode IoLink COM mode
  * @{
  */
 /** COM modes */
typedef enum {
  COM_1    = ((uint8_t)0x00),  /*   4.8 kbauds */
  COM_2    = ((uint8_t)0x01),  /*  38.4 kbauds */
  COM_3    = ((uint8_t)0x02),  /* 230.4 kbauds */
} iolinkComMode_t;

/**
  * @}
  */

/**
 * @brief  IOLink_Master driver structure definition
 */
typedef struct
{
  /**  Function pointer to Init */
  void (*Init)(void*);
  /** Function pointer to ReadID */
  uint16_t (*ReadID)(void);
  /** Function pointer to AttachErrorHandler */
  void(*AttachErrorHandler)(void (*callback)(uint16_t));
  /** Function pointer to AttachIrqInterrupt */
  void (*AttachIrqInterrupt)(void (*callback)(uint8_t));
  /** Function pointer to ErrorHandler */
  void (*ErrorHandler)(uint16_t);
  /** Function pointer to IrqInterruptHandler */
  void (*IrqInterruptHandler)(uint8_t);
  /** Function pointer to GetFwVersion */
  uint32_t (*GetFwVersion)(void);
  /** Function pointer to GetNbDevices */
  uint8_t (*GetNbDevices)(void);  
  /** Function pointer to SetNbDevices */  
  bool (*SetNbDevices)(uint8_t);
  /** Function pointer to GetResetState */
  uint8_t (*GetResetState)(uint8_t); 
  /** Function pointer to HandleReset */
  bool (*HandleReset)(uint8_t, uint8_t);       
  /** Function pointer to GetCQOuputEnable */
  uint8_t (*GetCQOuputEnable)(uint8_t); 
  /** Function pointer to SetCQOutputEnable */
  bool (*SetCQOutputEnable)(uint8_t, uint8_t);     
  /** Function pointer to GetLPlusEnable */
  uint8_t (*GetLPlusEnable)(uint8_t); 
  /** Function pointer to SetLPlusEnable */
  bool (*SetLPlusEnable)(uint8_t, uint8_t);       
  /** Function pointer to GetFaultState */
  uint8_t (*GetFaultState)(uint8_t);   
  /** Function pointer to GetOutIQState */
  uint8_t (*GetOutIQState)(uint8_t);     
  /** Function pointer to GetLPowerSupply */
  uint8_t (*GetLPowerSupply)(uint8_t);   
  /** Function pointer to SetLPowerSupply */
  bool (*SetLPowerSupply)(uint8_t, uint8_t);       
  /** Function pointer to GetComMode */
  iolinkComMode_t (*GetComMode)(uint8_t);     
  /** Function pointer to SelectComMode */
  bool (*SelectComMode)(uint8_t, iolinkComMode_t);     
  /** Function pointer to SendInCqData */
  bool (*SendInCqData)(uint8_t , uint8_t);     
  /** Function pointer to AttachOutCqDataHandler */
  void(*AttachOutCqDataHandler)(void (*callback)(uint8_t, uint8_t));
  /** Function pointer to OutCqDataHandler */
  void(*OutCqDataHandler)(uint8_t, uint8_t);
  /** Function pointer to ReadRegister */
  uint8_t (*ReadRegister)(uint8_t, uint8_t);
  /** Function pointer to WriteRegister */
  bool (*WriteRegister)(uint8_t, uint8_t, uint8_t);  
  /** Function pointer to GetStatus */
  uint16_t (*GetStatus)(uint8_t);    
  /** Function pointer to GetI2cFreq */
  uint32_t (*GetI2cFreq)(void);
  /** Function pointer to SetI2cFreq */
  bool (*SetI2cFreq)(uint32_t);  
}iolinkMasterDrv_t;

/**
 * @brief  IOLink_Device driver structure definition
 */
typedef struct
{
  /**  Function pointer to Init */
  void (*Init)(void*);
  /** Function pointer to ReadID */
  uint16_t (*ReadID)(void);
  /** Function pointer to AttachErrorHandler */
  void(*AttachErrorHandler)(void (*callback)(uint16_t));
  /** Function pointer to AttachFaultInterrupt */
  void (*AttachFaultInterrupt)(void (*callback)(uint8_t));
  /** Function pointer to AttachOLInterrupt */
  void (*AttachOLInterrupt)(void (*callback)(uint8_t));
  /** Function pointer to AttachOutIqDataHandler */
  void(*AttachOutIqDataHandler)(void (*callback)(uint8_t, uint8_t));
  /** Function pointer to ErrorHandler */
  void (*ErrorHandler)(uint16_t);
  /** Function pointer to FaultInterruptHandler */
  void (*FaultInterruptHandler)(uint8_t);
  /** Function pointer to OLInterruptHandler */
  void (*OLInterruptHandler)(uint8_t);
  /** Function pointer to OutIqDataHandler */
  void(*OutIqDataHandler)(uint8_t, uint8_t);
  /** Function pointer to GetFwVersion */
  uint32_t (*GetFwVersion)(void);
  /** Function pointer to GetNbDevices */
  uint8_t (*GetNbDevices)(void);  
  /** Function pointer to SetNbDevices */  
  bool (*SetNbDevices)(uint8_t);
  /** Function pointer to GetFaultState */
  uint8_t (*GetFaultState)(uint8_t);   
  /** Function pointer to GetEnable */
  uint8_t (*GetEnable)(uint8_t);     
  /** Function pointer to SetEnable */
  uint8_t (*SetEnable)(uint8_t, uint8_t);     
  /** Function pointer to GetComMode */
  iolinkComMode_t (*GetComMode)(uint8_t);     
  /** Function pointer to SelectComMode */
  bool (*SelectComMode)(uint8_t, iolinkComMode_t);     
  /** Function pointer to SendInData */
  bool (*SendInData)(uint8_t , uint8_t);     
}iolinkDeviceDrv_t;

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

#ifdef	 __cplusplus
}
#endif

#endif /* IOLINK_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
