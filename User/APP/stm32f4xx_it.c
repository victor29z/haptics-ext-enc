/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "ucos_ii.h"
#include "drivers.h"
#include "ch378.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern OS_EVENT* Sem_KEY2_EVT;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*  
void PendSV_Handler(void)
{
}
*/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

void SysTick_Handler(void)
{
	OS_CPU_SR  cpu_sr;
	
	OS_ENTER_CRITICAL();
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	OSTimeTick();      

	OSIntExit();
}

void USART1_IRQHandler(void)
{
	OSIntEnter();
	serial_isr(0);
	OSIntExit();    
}

void USART2_IRQHandler(void)
{
	OSIntEnter();    
	serial_isr(1);
	OSIntExit();    
}

#define RECV_STA_WAIT1ST	0x01
#define RECV_STA_WAIT2ND	0x02
#define RECV_STA_DATAH		0x03
#define RECV_STA_DATAL		0x04


#define EXTCMD_HEAD		0xaa
#define EXTCMD_SET_ENC1	0xe0
#define EXTCMD_SET_ENC2	0xe1
#define EXTCMD_SET_ENC3	0xe2
#define EXTCMD_SET_ENC4	0xe3

static uint8_t recv_sta = RECV_STA_WAIT1ST;

void USART3_IRQHandler(void)
{
	uint16_t ushTemp;
	static uint8_t enc;
	static uint16_t dat;
	OSIntEnter();
	
	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
	ushTemp = USART_ReceiveData(USART3); //取出来扔掉
	USART_ClearFlag(USART3, USART_FLAG_ORE);
	}
	
	if(USART_GetFlagStatus(USART3, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE 	: Noise Error interrupt
	USART_ClearFlag(USART3, USART_FLAG_NE);
	}
	
	
	if(USART_GetFlagStatus(USART3, USART_FLAG_FE) != RESET)
	{//同	@arg USART_IT_FE	 : Framing Error interrupt
	USART_ClearFlag(USART3, USART_FLAG_FE);
	}
	
	if(USART_GetFlagStatus(USART3, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE 	: Parity Error interrupt
	USART_ClearFlag(USART3, USART_FLAG_PE);
	}
	
	if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
	{
		ushTemp = USART_ReceiveData(USART3); 
		USART_ClearFlag(USART3, USART_FLAG_RXNE);
		switch(recv_sta){
			case RECV_STA_WAIT1ST:
				if(ushTemp == EXTCMD_HEAD) recv_sta = RECV_STA_WAIT2ND;
			break;
			case RECV_STA_WAIT2ND:
				if((ushTemp&0xfc) == 0xe0){
					 recv_sta = RECV_STA_DATAL;
					 dat = 0;
					 enc = ushTemp;
				}
			break;
			case RECV_STA_DATAL:
				dat = ushTemp;
				recv_sta = RECV_STA_DATAH;
			break;
			case RECV_STA_DATAH:
				dat = (ushTemp<<8) | dat;
				recv_sta = RECV_STA_WAIT1ST;
				switch (enc){
					case EXTCMD_SET_ENC1:
						SetEncoder(dat,0);
					break;
					case EXTCMD_SET_ENC2:
						SetEncoder(dat,1);
					break;
					case EXTCMD_SET_ENC3:
						SetEncoder(dat,2);
					break;
					case EXTCMD_SET_ENC4:
						SetEncoder(dat,3);
					break;
				}
			break;
				
			default:
				recv_sta = RECV_STA_WAIT1ST;
			
		}
	}
	OSIntExit();    
}
void UART4_IRQHandler(void)
{
	OSIntEnter();
	serial_isr(3);
	OSIntExit();    
}
void UART5_IRQHandler(void)
{
	OSIntEnter();
	serial_isr(4);
	OSIntExit();    
}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear the EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		mCH378Interrupt();
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line2);
		if((~(GetKeys()) & KEY1_MASK))
			CalEncoder();
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line3);
		if(~(GetKeys()) & KEY2_MASK)
			OSSemPost(Sem_KEY2_EVT); 

	}
}


void DMA1_Stream3_IRQHandler(void)  
{  
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) != RESET)   
    {  
        //清除标志位  
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);  
    }  
}  
     

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
