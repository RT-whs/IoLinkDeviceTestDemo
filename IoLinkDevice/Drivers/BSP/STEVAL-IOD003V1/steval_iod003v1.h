/**
  ******************************************************************************
  * @file   steval_iod003v1.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 8th, 2018
  * @brief   STEVAL_IOD003V1_Application
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
/** @defgroup STEVAL_IOD003V1_Exported_variables     STEVAL_IOD003V1 Exported variables 
  * @{
  * @brief Exported variables 
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STEVAL_IOD003V1_H
#define STEVAL_IOD003V1_H
    
/* Includes ------------------------------------------------------------------*/
#include "iolink.h"

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_nucleo.h"
#elif defined (USE_STM32L0XX_NUCLEO)
#include "stm32l0xx_nucleo.h"
#else
#error "No STM32 is defined in file steval_iod003v1.h"
#endif

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup STEVAL_IOD003V1 STEVAL_IOD003V1
 * @{
 */

/** @addtogroup STEVAL_IOD003V1_IO IO
 * @{
 */

/** @addtogroup STEVAL_IOD003V1_IO_Public_Variables Public variables
 * @{
 */
   
/** UART handler declaration */
extern UART_HandleTypeDef gUartHandle;
    
/** UART rx buffer */
extern uint8_t uartRxBuffer;
/**
  * @}
  */

   
/** @addtogroup STEVAL_IOD003V1_IO_Public_Constants Public constants
 * @{
 */

/******************************************************************************/
/* Dependent plateform definitions                                            */
/******************************************************************************/

#ifdef USE_STM32F4XX_NUCLEO

/** Interrupt line used for the Diag pin */
#define BSP_IOLINK_DEVICE_BOARD_DIAG_IRQn       (EXTI4_IRQn)

/** Interrupt line used for the OL pin */
#define BSP_IOLINK_DEVICE_BOARD_OL_IRQn         (EXTI9_5_IRQn)


/** UART gpio alternate for IN i/q */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_IN_GPIO_AF        (GPIO_AF7_USART1)     

/** UART gpio alternate for OUT i/q */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_OUTIQ_GPIO_AF        (GPIO_AF7_USART1)     


#elif defined (USE_STM32L0XX_NUCLEO)

/** Interrupt line used for the Diag pin */
#define BSP_IOLINK_DEVICE_BOARD_DIAG_IRQn       (EXTI4_15_IRQn)

/** Interrupt line used for the OL pin */
#define BSP_IOLINK_DEVICE_BOARD_OL_IRQn         (EXTI4_15_IRQn)

/** UART gpio alternate for IN */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_IN_GPIO_AF        (GPIO_AF0_USART1)     

/** UART gpio alternate for OUT i/q */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_OUTIQ_GPIO_AF        (GPIO_AF4_USART1)     


#else
#error "No STM32 is defined in file steval_iod003v1.h"
#endif
/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

/** GPIO Pin used for the L6362A Enable pin */
#define BSP_IOLINK_DEVICE_BOARD_EN_PIN          (GPIO_PIN_7)
/** GPIO port used for the L6362A Enable pin */
#define BSP_IOLINK_DEVICE_BOARD_EN_PORT         (GPIOC)

/** GPIO Pin used to handle the Diag pin */
#define BSP_IOLINK_DEVICE_BOARD_DIAG_PIN        (GPIO_PIN_4)
/** GPIO Pprt used to handle the Diag pin */
#define BSP_IOLINK_DEVICE_BOARD_DIAG_PORT       (GPIOA)

/** GPIO Pin used to handle the OL pin */
#define BSP_IOLINK_DEVICE_BOARD_OL_PIN          (GPIO_PIN_6)
/** GPIO Pprt used to handle the OL pin  */
#define BSP_IOLINK_DEVICE_BOARD_OL_PORT         (GPIOA)

/** GPIO Pin used for the IN pin */
#define BSP_IOLINK_DEVICE_BOARD_IN_PIN          (GPIO_PIN_6)
/** GPIO port used for the IN pin */
#define BSP_IOLINK_DEVICE_BOARD_IN_PORT         (GPIOB)

/** GPIO Pin used for the OUTIQ pin of the L6362A*/
#define BSP_IOLINK_DEVICE_BOARD_OUT_IQ_PIN      (GPIO_PIN_10)
/** GPIO port used for the OUTIQ pin of the L6362A */
#define BSP_IOLINK_DEVICE_BOARD_OUT_IQ_PORT     (GPIOA)

/** Used UART for IN & OUT i/q */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_IN_OUTIQ                (USART1)   

/** UART clock enable for IN & OUT i/q */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_IN_OUTIQ_CLK_ENABLE()   __HAL_RCC_USART1_CLK_ENABLE()      

/** UART clock disable for IN & OUT i/q */
#define BSP_IOLINK_DEVICE_BOARD_UARTX_IN_OUTIQ_CLK_DISABLE()  __HAL_RCC_USART1_CLK_DISABLE()      

/** Uart IRQn for IN & OUT i/q */
#define  BSP_IOLINK_USARTx_IN_OUTIQ_IRQn                      (USART1_IRQn)

/** Uart IRQ hanlder  for IN & OUT i/q */
#define BSP_IOLINK_USARTx_IN_OUTIQ_IRQHandler                 USART1_IRQHandler
   
/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup STEVAL_IOD003V1_Public_variables   STEVAL_IOD003V1 Public variables
 * @{
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

#endif /*STEVAL_IOD003V1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
