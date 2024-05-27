/**
  ******************************************************************************
  * @file    l6362.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 8th, 2018
  * @brief   L6362A driver (IO-Link communication device transceiver IC)
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

/* Includes ------------------------------------------------------------------*/
#include "l6362a.h"
    
/* Private constants  ---------------------------------------------------------*/
    
/** @addtogroup BSP
  * @{
  */   
   
/** @defgroup L6362A L6362A
  * @{
  */   

/* Private constants ---------------------------------------------------------*/    

/** @defgroup L6362A_Private_Constants L6362A Private Constants
  * @{
  */   

/** Error while initialising the UART */
#define L6362A_ERROR_0   (0x8000U)   


/**
  * @}
  */ 
    
/* Private variables ---------------------------------------------------------*/

/** @defgroup L6362A_Private_Variables L6362A Private Variables
  * @{
  */       

/** Function pointer to error handler call back */
static void (*pErrorHandlerCallback)(uint16_t);
/** Function pointer to Diag interrupt call back */
static void (*pDiagInterruptCallback)(uint8_t);
/** Function pointer to OL interrupt call back */
static void (*pOLInterruptCallback)(uint8_t);
/** Function pointer to OUT IQ data handler call back */
static void (*pOutIqDataCallback)(uint8_t, uint8_t);

static uint16_t l6362DriverInstance = 0U;
static uint8_t numberOfDevices;

/** L6362A Driver Paramaters structure */
static driverParams_t driverPrm;

/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/

/** @defgroup L6362A_Private_functions L6362A Private functions
  * @{
  */  
void L6362a_ErrorHandler(uint16_t error);
void L6362a_DiagInterruptHandler(uint8_t deviceId);                      
void L6362a_OLInterruptHandler(uint8_t deviceId);                      
void L6362a_OutIqDataHandler(uint8_t deviceId, uint8_t data);                      
void L6362a_SetDeviceParamsToPredefinedValues(uint8_t deviceId);
void L6362a_SetDeviceParamsToGivenValues(uint8_t deviceId, const l6362a_Init_t *pInitPrm);

/**
  * @}
  */ 

/** @defgroup L6362A_Exported_Variables L6362A Exported Variables
  * @{
  */       

/** L6362A iolink device driver functions pointer structure  */
static iolinkDeviceDrv_t   l6362Drv =
{
  &L6362a_Init,                         /* void (*Init)(void*); */
  &L6362a_ReadId,                       /* uint16_t (*ReadID)(void); */
  &L6362a_AttachErrorHandler,           /* void (*AttachErrorHandler)(void (*callback)(uint16_t)); */
  &L6362a_AttachDiagInterrupt,          /* void (*AttachFaultInterrupt)(void (*callback)(uint8_t)); */
  &L6362a_AttachOLInterrupt,            /* void (*AttachOLInterrupt)(void (*callback)(uint8_t)); */
  &L6362a_AttachOutIqDataHandler,       /* void(*AttachOutIqDataHandler)(void (*callback)(uint8_t, uint8_t)); */
  &L6362a_ErrorHandler,                 /* void (*ErrorHandler)(uint16_t); */
  &L6362a_DiagInterruptHandler,         /* void (*FaultInterruptHandler)(uint8_t); */
  &L6362a_OLInterruptHandler,           /* void (*OLInterruptHandler)(uint8_t); */
  &L6362a_OutIqDataHandler,             /* void(*OutIqDataHandler)(uint8_t, uint8_t); */
  &L6362a_GetFwVersion,                 /* uint32_t (*GetFwVersion)(void); */
  &L6362a_GetNbDevices,                 /* uint8_t (*GetNbDevices)(void); */
  &L6362a_SetNbDevices,                 /* bool (*SetNbDevices)(uint8_t); */
  &L6362a_GetFaultState,                /* uint8_t (*GetFaultState)(uint8_t); */
  &L6362a_GetEnable,                    /* uint8_t (*GetEnable)(uint8_t); */
  &L6362a_SetEnable,                    /* bool (*SetEnable)(uint8_t, uint8_t); */
  &L6362a_GetComMode,                   /* iolinkComMode_t (*GetComMode)(uint8_t);  */  
  &L6362a_SelectComMode,                /* bool (*SelectComMode)(uint8_t, iolinkComMode_t);  */  
  &L6362a_SendInData,                   /* bool (*SendInData)(uint8_t , uint8_t);  */  

 };     
  
/**
  * @}
  */ 
    
/** @defgroup L6362A_Library_Functions L6362A Library Functions
  * @{
  */   

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error handler
 * @retval None
 **********************************************************/
void L6362a_AttachErrorHandler(void (*callback)(uint16_t))
{
  pErrorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief  Attaches a user callback to the Diag interrupt
 * The call back will be then called each time the Diag 
 * (Interrupt request signal) pin will be pulled 
 * down/pull up 
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
void L6362a_AttachDiagInterrupt(void (*callback)(uint8_t))
{
  pDiagInterruptCallback = (void (*)(uint8_t))callback;
}       
/******************************************************//**
 * @brief  Attaches a user callback to the OL interrupt
 * The call back will be then called each time the OKL 
 * (Interrupt request signal) pin will be pulled 
 * down/pull up 
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
void L6362a_AttachOLInterrupt(void (*callback)(uint8_t))
{
  pOLInterruptCallback = (void (*)(uint8_t))callback;
}       
/******************************************************//**
 * @brief  Attaches a user callback to the OUT IQ interrupt
 * The call back will be then called each time a data is 
 * received on OUT IQ
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
void L6362a_AttachOutIqDataHandler(void (*callback)(uint8_t, uint8_t))
{
  pOutIqDataCallback = (void (*)(uint8_t, uint8_t))callback;
}       

/******************************************************//**
 * @brief  Get the com Mode of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval comMode COM_MODE_1, COM_MODE_2 or COM_MODE_3
 **********************************************************/
iolinkComMode_t L6362a_GetComMode(uint8_t deviceId)
{
  return (driverPrm.device[deviceId].comMode);
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval L6362A_FW_VERSION
 **********************************************************/
uint32_t L6362a_GetFwVersion(void)
{
  return (L6362A_FW_VERSION);
}

/******************************************************//**
 * @brief Return IO Link Device handle 
 *(pointer to the L6362A IO Link device driver structure)
 * @retval Pointer to the iolinkDeviceDrv_t structure
 **********************************************************/
iolinkDeviceDrv_t* L6362a_GetIOLinkDeviceHandle(void)
{
  return (&l6362Drv);
}
/******************************************************//**
 * @brief  Returns the number of devices
 * @retval number of devices
 **********************************************************/
uint8_t L6362a_GetNbDevices(void)
{
  return (numberOfDevices);
}

/******************************************************//**
 * @brief  Get the Fault state (Diag and OL states) 
 * of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval state 0: no fault, 1: fault repoted by Diag pin 
 *               2: fault repoted by OL pin 
 *               3: fault repoted by both Diag and OL pin 
 **********************************************************/
uint8_t L6362a_GetFaultState(uint8_t deviceId)
{
  uint8_t state = 0U; 
  if (deviceId < l6362DriverInstance)
  {
    if (L6362a_Board_GetDiagPinState(deviceId) == 0U)
    {
      state += 1U;
    }
    if (L6362a_Board_GetOLPinState(deviceId) == 0U)
    {
      state += 2U;
    }
  }
  return (state);
}

/******************************************************//**
 * @brief  Get the Enable state of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval state (1 if enable, 0 if disable)
 **********************************************************/
uint8_t L6362a_GetEnable(uint8_t deviceId)
{
  uint8_t state = 0U; 
  if (deviceId < l6362DriverInstance)
  {
    state = L6362a_Board_GetEnablePinState(deviceId);
  }
  return (state);
}

/******************************************************//**
 * @brief Starts a new L6362A instance 
 * @param[in] pInit pointer to the initialization data
 * @retval None
 **********************************************************/

void L6362a_Init(void* pInit)
{
  /* Set all registers and context variables to the predefined values from l6362_target_config.h */
  if (pInit == (void*)0)
  {
    L6362a_SetDeviceParamsToPredefinedValues((uint8_t)l6362DriverInstance);
  }
  else
  {
    L6362a_SetDeviceParamsToGivenValues((uint8_t)l6362DriverInstance, pInit);
  }

  /* Initialise the GPIOs */
  L6362a_Board_GpioInit((uint8_t)l6362DriverInstance);

  /* Initialise the UART */
  if (L6362a_Board_UartInit((uint8_t)l6362DriverInstance) != 0U)
  {
    /* UART initialization Error */
    L6362a_ErrorHandler(L6362A_ERROR_0);    
  }

  /* Let a delay after reset */
  L6362a_Board_Delay(1U);                     

  l6362DriverInstance++;
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the L6362A Driver Instance
 **********************************************************/
uint16_t L6362a_ReadId(void)
{
  return (l6362DriverInstance);
}

/******************************************************//**
 * @brief  Select the com Mode of the specifie device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] comMode COM_MODE_1, COM_MODE_2 or COM_MODE_3
 * @retval TRUE if successfull, FALSE if failure
 **********************************************************/
bool L6362a_SelectComMode(uint8_t deviceId, iolinkComMode_t comMode)
{
  bool status = FALSE; 
  if (deviceId < l6362DriverInstance)
  {
    if (driverPrm.device[deviceId].comMode != comMode)
    {
      uint32_t baudrate;
      switch(comMode)
      {
        case COM_1:  
          baudrate = 4800U;           
          break; 
        case COM_2:
          baudrate = 38400U;
          break;          
        case COM_3:
        default:
          baudrate = 230400U;
          break;
      }
      if (L6362a_Board_UartSetBaudrate(deviceId, baudrate) == 0U)
      {
        status = TRUE;
        driverPrm.device[deviceId].comMode = comMode;
      }
    }
    else
    {
      status = TRUE;
    } 
  }
  return (status);
}


/******************************************************//**
 * @brief  Select the com Mode of the specifie device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] data byte to transmit
 * @retval TRUE if successfull, FALSE if failure
 **********************************************************/
bool L6362a_SendInData(uint8_t deviceId, uint8_t data)
{
  bool status; 
  
  status = (L6362a_Board_UartSendData(deviceId, data) != 0U);
  
  return (status);
}


/******************************************************//**
 * @brief  Enable or disable the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] enable 1 to enable, 0 to disable
 * @retval TRUE if successfull, FALSE if failure
 **********************************************************/
bool L6362a_SetEnable(uint8_t deviceId, uint8_t enable)
{
  bool status = FALSE; 
  if (deviceId < l6362DriverInstance)
  {
    if (enable != 0U)
    {
      status = (L6362a_Board_SetEnablePin(deviceId) == 0U);
    }
    else
    {
      status = (L6362a_Board_ResetEnablePin(deviceId) == 0U);
    }
  }
  return (status);
}


/******************************************************//**
 * @brief  Sets the number of devices to be used 
 * @param[in] nbDevices (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of 
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool L6362a_SetNbDevices(uint8_t nbDevices)
{
  bool status = FALSE; 
  if (nbDevices <= MAX_NUMBER_OF_DEVICES)
  {
    l6362DriverInstance = 0U;
    numberOfDevices = nbDevices;
    status = TRUE;
  }
  return (status);
}

/**
  * @}
  */


/** @addtogroup L6362A_Private_functions
  * @{
  */  

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6362a_ErrorHandler(uint16_t error)
{
  if (pErrorHandlerCallback != (void *)0)
  {
    (void) pErrorHandlerCallback(error);
  }
  else   
  {
    while(TRUE)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the fault interrupt which calls 
 * the user callback (if defined)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void L6362a_DiagInterruptHandler(uint8_t deviceId)
{
  if (pDiagInterruptCallback != (void *)0)
  {
    pDiagInterruptCallback(deviceId);
  }
}

/******************************************************//**
 * @brief  Handlers of the OL interrupt which calls 
 * the user callback (if defined)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void L6362a_OLInterruptHandler(uint8_t deviceId)
{
  if (pOLInterruptCallback != (void *)0)
  {
    pOLInterruptCallback(deviceId);
  }
}

/******************************************************//**
 * @brief  Handlers of the OUT IQ data which calls 
 * the user callback (if defined)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] data OUT IQ received data
 * @retval None
 **********************************************************/
void L6362a_OutIqDataHandler(uint8_t deviceId, uint8_t data)
{
  if (pOutIqDataCallback != (void *)0)
  {
    pOutIqDataCallback(deviceId, data);
  }
}

/******************************************************//**
 * @brief  Set the parameters of the device to values of pInitPrm structure
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param pInitPrm pointer to a structure containing the initial device parameters 
 * @retval None
 **********************************************************/
void L6362a_SetDeviceParamsToGivenValues(uint8_t deviceId, const l6362a_Init_t *pInitPrm)
{
  driverPrm.device[deviceId].comMode = pInitPrm->comMode;
}

/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values 
 * from l6362_target_config.h
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void L6362a_SetDeviceParamsToPredefinedValues(uint8_t deviceId)
{
  driverPrm.device[deviceId].comMode = L6362A_CONF_PARAM_COM_MODE;
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
