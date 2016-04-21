/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#ifdef STM32L1XX_MD
#include "stm32l1xx.h"
#else
#include "stm32f10x.h"
#endif /* STM32L1XX_MD */

#include "platform_config.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "stm32_eval.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Receive_Buffer[64];
extern __IO uint8_t PrevXferComplete;
extern void cmd_handler(const void* data);
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
  BitAction Led_State;

  /* Read received data (2 bytes) */  
  USB_SIL_Read(EP1_OUT, Receive_Buffer);
    
  cmd_handler(Receive_Buffer);
    
  if (0 && PrevXferComplete)
  {
      /* Write the descriptor through the endpoint */
      USB_SIL_Write(EP1_IN, (uint8_t*) Receive_Buffer, 64);  
      
#ifndef STM32F10X_CL
      SetEPTxValid(ENDP1);
#endif /* STM32F10X_CL */
  }
 
#ifndef STM32F10X_CL   
  SetEPRxStatus(ENDP1, EP_RX_VALID);
#endif /* STM32F10X_CL */
 
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  PrevXferComplete = 1;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

