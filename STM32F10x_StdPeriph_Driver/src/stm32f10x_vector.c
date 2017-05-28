/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_vector.c
* Author             : MCD Application Team
* Date First Issued  : 02/19/2007
* Description        : This file contains the vector table for STM32F10x.
*                      After Reset the Cortex-M3 processor is in Thread mode,
*                      priority is Privileged, and the Stack is set to Main.
********************************************************************************
* History:
* 04/02/2007: V0.2
* 02/19/2007: V0.1
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_it.h"

extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );
extern void vTimer2IntHandler( void );
extern void vUARTInterruptHandler( void );
extern void vPortSVCHandler( void );

/* Private typedef -----------------------------------------------------------*/
typedef void( *intfunc )( void );
typedef union { intfunc __fun; void * __ptr; } intvec_elem;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


#pragma language=extended
#pragma segment="CSTACK"

void __iar_program_start( void );

#pragma location = ".intvec"
/* STM32F10x Vector Table entries */
const intvec_elem __vector_table[] =
{
  { .__ptr = __sfe( "CSTACK" ) },
  &__iar_program_start,
             NMI_Handler,
             HardFault_Handler,
             MemManage_Handler,
             BusFault_Handler,
             UsageFault_Handler,      
             0,
             0,                  
             0,              
             0,    
             SVC_Handler,    
             DebugMon_Handler,
             0,
             PendSV_Handler,        
             SysTick_Handler,      
     
             WWDG_IRQHandler,         
             PVD_IRQHandler,           
             TAMPER_IRQHandler,       
             RTC_IRQHandler,          
             FLASH_IRQHandler,        
             EXTI0_IRQHandler,          
             EXTI1_IRQHandler,        
             EXTI2_IRQHandler,          
             EXTI3_IRQHandler,         
             EXTI4_IRQHandler,          
             DMA1_Channel1_IRQHandler,  
             DMA1_Channel2_IRQHandler,  
             DMA1_Channel3_IRQHandler,  
             DMA1_Channel4_IRQHandler, 
             DMA1_Channel5_IRQHandler,  
             DMA1_Channel6_IRQHandler,  
             DMA1_Channel7_IRQHandler,  
             ADC1_2_IRQHandler,         
             USB_HP_CAN1_TX_IRQHandler,  
             USB_LP_CAN1_RX0_IRQHandler, 
             CAN1_RX1_IRQHandler,       
             CAN1_SCE_IRQHandler,       
             EXTI9_5_IRQHandler,        
             TIM1_BRK_IRQHandler,       
             TIM1_UP_IRQHandler,      
             TIM1_TRG_COM_IRQHandler,  
             TIM1_CC_IRQHandler,       
             TIM2_IRQHandler,           
             TIM3_IRQHandler,          
             TIM4_IRQHandler,           
             I2C1_EV_IRQHandler,        
             I2C1_ER_IRQHandler,       
             I2C2_EV_IRQHandler,       
             I2C2_ER_IRQHandler,       
             SPI1_IRQHandler,          
             SPI2_IRQHandler,          
             USART1_IRQHandler,        
             USART2_IRQHandler,        
             USART3_IRQHandler,         
             EXTI15_10_IRQHandler,      
             RTCAlarm_IRQHandler,      
             USBWakeUp_IRQHandler,     
             TIM8_BRK_IRQHandler,      
             TIM8_UP_IRQHandler,       
             TIM8_TRG_COM_IRQHandler,   
             TIM8_CC_IRQHandler,        
             ADC3_IRQHandler,         
             FSMC_IRQHandler,           
             SDIO_IRQHandler,           
             TIM5_IRQHandler,          
             SPI3_IRQHandler,           
             UART4_IRQHandler,         
             UART5_IRQHandler,         
             TIM6_IRQHandler,           
             TIM7_IRQHandler,           
             DMA2_Channel1_IRQHandler, 
             DMA2_Channel2_IRQHandler, 
             DMA2_Channel3_IRQHandler,  
             DMA2_Channel4_5_IRQHandler,
};
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/

