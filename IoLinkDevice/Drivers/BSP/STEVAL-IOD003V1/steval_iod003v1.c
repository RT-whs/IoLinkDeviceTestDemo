/**
  ******************************************************************************
  * @file    steval_iod003v1.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 8th, 2018
  * @brief   STEVAL_IOD003V1_application.
  * This file provides firmware functions which are Application Oriented
  ==============================================================================    
 
           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "steval_iod003v1.h"
    
/** @addtogroup BSP     BSP
  * @{
  * @brief 
  */ 


/** @defgroup STEVAL_IOD003V1     STEVAL_IOD003V1  
  * @brief   
  *@{
  */

/** @addtogroup STEVAL_IOD003V1_Private_Constants STEVAL_IOD003V1 Private Constants
  * @{
  */   


/**
  * @}
  */ 
    
/** @defgroup STEVAL_IOD003V1_Private_variables   STEVAL_IOD003V1 Private variables
  * @brief  
  * @{
  */    
    /** UART handler declaration */
    UART_HandleTypeDef gUartHandle;
    /** UART tx buffer */
    static uint8_t uartTxBuffer;
    /** UART rx buffer */
    uint8_t uartRxBuffer;
/**
  * @} 
  */
    
/** @defgroup STEVAL_IOD003V1_Public_variables   STEVAL_IOD003V1 Public variables
  * @brief  
  * @{
  */    

/**
  * @} 
  */

/**
  * @} 
  */

/** @defgroup STEVAL_IOD003V1_Private_Function_Prototypes STEVAL_IOD003V1 Private Function Prototypes
 * @{
 */
void L6362a_Board_Delay(uint32_t delay);         /* Delay of the requested number of milliseconds */
uint8_t L6362a_Board_GetDiagPinState(uint8_t deviceId); /* Get diag pin state */
uint8_t L6362a_Board_GetEnablePinState(uint8_t deviceId); /* Get enable pin state */
uint8_t L6362a_Board_GetOLPinState(uint8_t deviceId); /* Get OL (overload) pin state */
void L6362a_Board_GpioInit(uint8_t deviceId);     /* Initialise the GPIOs used for IOD003s devices */
uint8_t L6362a_Board_ResetEnablePin(uint8_t deviceId);  /* Set the gpio of the enable pin*/
uint8_t L6362a_Board_SetEnablePin(uint8_t deviceId);   /* Reset  the gpio of the enable pin*/
uint8_t L6362a_Board_UartInit(uint8_t deviceId);   /* Initialise the UART used for L6362A */
uint8_t L6362a_Board_UartSendData(uint8_t deviceId, uint8_t data);   /* send data to L6362A via uart (In data) */
uint8_t L6362a_Board_UartSetBaudrate(uint8_t deviceId, uint32_t baudrate); /* Set the UART baudrate used for L6362A */

/**
  * @} 
  */

/** @defgroup STEVAL_IOD003V1_Private_Function STEVAL_IOD003V1 Private Function 
 * @{
 */

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void L6362a_Board_Delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/******************************************************//**
 * @brief  Reset the Enable pin of the specified L6362A (output disable)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @retval HAL_OK if Uart activation is OK, HAL_ERROR else
 * @note The reception on the associated uart is reactivated
  **********************************************************/
uint8_t L6362a_Board_ResetEnablePin(uint8_t deviceId)
{
  HAL_StatusTypeDef status;
  if (deviceId == 0U)
  {
    HAL_GPIO_WritePin(BSP_IOLINK_DEVICE_BOARD_EN_PORT, BSP_IOLINK_DEVICE_BOARD_EN_PIN, GPIO_PIN_RESET);   
    /* Enable Rx Uart when Enable pin is Low */ 
    __HAL_UART_FLUSH_DRREGISTER(&gUartHandle);
    __HAL_UART_CLEAR_OREFLAG(&gUartHandle);
    status = HAL_UART_Receive_IT(&gUartHandle, &uartRxBuffer, 1U);
  }
  else
  {
    status = HAL_ERROR;
  }
  return (uint8_t) status;      
}

/******************************************************//**
 * @brief  Set the Enable pin of the specified L6362A (output enable)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @retval HAL_OK if Uart deactivation is OK, HAL_KO else
 * @note The reception on the associated uart is aborted
  **********************************************************/
uint8_t L6362a_Board_SetEnablePin(uint8_t deviceId)
{
  HAL_StatusTypeDef status;
  if (deviceId == 0U)
  {
    HAL_GPIO_WritePin(BSP_IOLINK_DEVICE_BOARD_EN_PORT, BSP_IOLINK_DEVICE_BOARD_EN_PIN, GPIO_PIN_SET);   
    status = HAL_UART_AbortReceive(&gUartHandle);
  }
  else
  {
    status = HAL_ERROR;
  }
  return (uint8_t) status;     
}

/******************************************************//**
 * @brief  Get the Diag pin state of the specified L6362A
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @retval status of the Diagnostic pin
  **********************************************************/
uint8_t L6362a_Board_GetDiagPinState(uint8_t deviceId)
{
  uint8_t state;
  
 if (deviceId == 0U)
  {
    state = (uint8_t)HAL_GPIO_ReadPin(BSP_IOLINK_DEVICE_BOARD_DIAG_PORT, BSP_IOLINK_DEVICE_BOARD_DIAG_PIN); 
  }
  else
  {
      /* Ignore the request */ 
    state = 0U;
  }  
  
  return (state);
}

/******************************************************//**
 * @brief  Get the Diag pin state of the specified L6362A
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @retval status of the Diagnostic pin
  **********************************************************/
uint8_t L6362a_Board_GetEnablePinState(uint8_t deviceId)
{
  uint8_t state;
  
 if (deviceId == 0U)
  {
    state = (uint8_t)HAL_GPIO_ReadPin(BSP_IOLINK_DEVICE_BOARD_EN_PORT, BSP_IOLINK_DEVICE_BOARD_EN_PIN); 
  }
  else
  {
      /* Ignore the request */ 
    state = 0U;
  }  
  
  return (state);
}

/******************************************************//**
 * @brief  Get the OL (overload) pin state of the specified L6362A
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @retval status of the OL (overload) pin
  **********************************************************/
uint8_t L6362a_Board_GetOLPinState(uint8_t deviceId)
{
  uint8_t state;
  
 if (deviceId == 0U)
  {
    state = (uint8_t)HAL_GPIO_ReadPin(BSP_IOLINK_DEVICE_BOARD_OL_PORT, BSP_IOLINK_DEVICE_BOARD_OL_PIN); 
  }
  else
  {
      /* Ignore the request */ 
    state = 0U;
  }  
  
  return (state);
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the L6362As
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
  **********************************************************/
void L6362a_Board_GpioInit(uint8_t deviceId)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  
  if (deviceId == 0U)
  {
    /* Configure the L6362A - En (Enable) pin--------------------------*/
    GPIO_InitStruct.Pin = BSP_IOLINK_DEVICE_BOARD_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_IOLINK_DEVICE_BOARD_EN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BSP_IOLINK_DEVICE_BOARD_EN_PORT, BSP_IOLINK_DEVICE_BOARD_EN_PIN, GPIO_PIN_RESET); 

    /* Configure the L6362A - Diag pin  --------------------------*/
    GPIO_InitStruct.Pin = BSP_IOLINK_DEVICE_BOARD_DIAG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BSP_IOLINK_DEVICE_BOARD_DIAG_PORT, &GPIO_InitStruct);
    
    /* Set Priority of Exti line Interrupt used for the Diag interrupt */ 
    HAL_NVIC_SetPriority(BSP_IOLINK_DEVICE_BOARD_DIAG_IRQn, 2U, 0U);
      
    /* Enable the Exti line Interrupt used for the Diag interrupt*/
    HAL_NVIC_EnableIRQ(BSP_IOLINK_DEVICE_BOARD_DIAG_IRQn);    

    /* Configure the L6362A - OL pin  (Overload pin) --------------------------*/
    GPIO_InitStruct.Pin = BSP_IOLINK_DEVICE_BOARD_OL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BSP_IOLINK_DEVICE_BOARD_OL_PORT, &GPIO_InitStruct);
    
    /* Set Priority of Exti line Interrupt used for the OL interrupt */ 
    HAL_NVIC_SetPriority(BSP_IOLINK_DEVICE_BOARD_OL_IRQn, 2U, 0U);
      
    /* Enable the Exti line Interrupt used for the OL interrupt*/
    HAL_NVIC_EnableIRQ(BSP_IOLINK_DEVICE_BOARD_OL_IRQn);        
  }
  else
  {
    /* Ignore the request */ 
  }  
  
  /* Let a delay after init */
  L6362a_Board_Delay(1U); 
}

/******************************************************//**
 * @brief  Initiliases the Uart used by the L6362s
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @retval HAL_OK if Uart initialisation is OK, HAL_KO else
  **********************************************************/
uint8_t L6362a_Board_UartInit(uint8_t deviceId)
{
  HAL_StatusTypeDef status;
  UART_HandleTypeDef *pUartHandle;
  
  if (deviceId == 0U)
  {
    uint8_t pinState;
    pUartHandle = &gUartHandle;
    pUartHandle->Instance  = BSP_IOLINK_DEVICE_BOARD_UARTX_IN_OUTIQ;
    pUartHandle->Init.BaudRate = 230400U;
    pUartHandle->Init.WordLength = UART_WORDLENGTH_8B; 
    pUartHandle->Init.StopBits = UART_STOPBITS_1;
    pUartHandle->Init.Parity = UART_PARITY_NONE;
    pUartHandle->Init.Mode = UART_MODE_TX_RX;
    pUartHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    pUartHandle->Init.OverSampling = UART_OVERSAMPLING_16;
    
    status = HAL_UART_Init(pUartHandle);
    
    pinState = L6362a_Board_GetEnablePinState(deviceId);
    if ((status == HAL_OK)&&(pinState == 0U))
    {
      __HAL_UART_FLUSH_DRREGISTER(&gUartHandle); 
      __HAL_UART_CLEAR_OREFLAG(&gUartHandle);
      status = HAL_UART_Receive_IT(pUartHandle, &uartRxBuffer, 1U);
    }    
  }
  else
  {  
    status = HAL_ERROR;
  }

  return (uint8_t) status;  
}

/******************************************************//**
 * @brief  Send Uart data to L6362 (In)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @param[in] data byte to transmit
 * @retval HAL_OK if Uart initialisation is OK, HAL_KO else
  **********************************************************/
uint8_t L6362a_Board_UartSendData(uint8_t deviceId, uint8_t data)
{
  HAL_StatusTypeDef status;
  
  if (deviceId == 0U)
  {
    uartTxBuffer  = data;
    status = HAL_UART_Transmit_IT(&gUartHandle, &uartTxBuffer, 1U);
  }
  else
  {  
    status = HAL_ERROR;
  }
  
  return (uint8_t) status;  
}
/******************************************************//**
 * @brief  Set the Uart baudrate
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1 )
 * @param[in] baudrate to use in baud
 * @retval HAL_OK if Uart initialisation is OK, HAL_KO else
  **********************************************************/
uint8_t L6362a_Board_UartSetBaudrate(uint8_t deviceId, uint32_t baudrate)
{
  HAL_StatusTypeDef status;
  
  UART_HandleTypeDef *pUartHandle;
  
  if (deviceId == 0U)
  {
    /* Disable output enable and add delay to avoid dummy  transmission */
    uint8_t pinState = L6362a_Board_GetEnablePinState(deviceId);
    if (pinState != 0)
    {
      L6362a_Board_ResetEnablePin(deviceId);
      L6362a_Board_Delay(1U); 
    }
    
    pUartHandle = &gUartHandle;
    status = HAL_UART_DeInit(pUartHandle);
    if (status == HAL_OK)
    {
      pUartHandle->Init.BaudRate = baudrate;
      status = HAL_UART_Init(pUartHandle);  
      
      /* Add delay to avoid dummy RX reception */
      L6362a_Board_Delay(1U); 
      
      if (status == HAL_OK)
      {
        __HAL_UART_FLUSH_DRREGISTER(&gUartHandle);
        __HAL_UART_CLEAR_OREFLAG(&gUartHandle);
        status = HAL_UART_Receive_IT(pUartHandle, &uartRxBuffer, 1U);
      }
    }
    if (pinState != 0)
    {
      /* Re-enable output if required */
      L6362a_Board_SetEnablePin(deviceId);
    }
  }
  else
  {  
    status = HAL_ERROR;
  }

  return (uint8_t) status;      
}
/**
  * @}  
  */
/**
  * @} 
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
