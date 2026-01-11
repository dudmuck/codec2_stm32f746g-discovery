/**
  ******************************************************************************
  * @file    BSP/Src/stm32f7xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
#ifdef ENABLE_LR20XX
#include "pinDefs_lr20xx.h"
#endif
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif


/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern DMA_HandleTypeDef   hdma;
/*DMA status declared in "sdram_dma.c" file */
extern uint32_t uwDMA_Transfer_Complete;
/* SAI handler declared in "stm32746g_discovery_audio.c" file */
extern SAI_HandleTypeDef haudio_out_sai;
/* SAI handler declared in "stm32746g_discovery_audio.c" file */
extern SAI_HandleTypeDef haudio_in_sai;
/* SDRAM handler declared in "stm32746g_discovery_sdram.c" file */
extern SDRAM_HandleTypeDef sdramHandle;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
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

/* FreeRTOS provides SVC_Handler and PendSV_Handler via port.c assembly code.
 * These are only needed when NOT using FreeRTOS. */
#ifndef USE_FREERTOS
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
#endif /* !USE_FREERTOS */

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
#ifdef USE_FREERTOS
  /* Only call FreeRTOS tick handler if scheduler has started */
  extern void xPortSysTickHandler(void);
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
#endif
}

#ifdef USE_FREERTOS
/**
  * @brief  FreeRTOS stack overflow hook - called when stack overflow is detected
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  /* Print task name before halting - use direct UART write to avoid further stack usage */
  volatile char *msg = "\r\n*** STACK OVERFLOW: ";
  volatile char *name = pcTaskName;
  /* Direct output via ITM or just halt with task name in register for debugger */
  taskDISABLE_INTERRUPTS();
  /* Keep name in register for GDB inspection */
  __asm volatile("mov r0, %0" : : "r"(name));
  for (;;);
}

/**
  * @brief  FreeRTOS malloc failed hook - called when pvPortMalloc fails
  */
void vApplicationMallocFailedHook(void)
{
  /* Infinite loop on malloc failure */
  taskDISABLE_INTERRUPTS();
  for (;;);
}

/**
  * @brief  FreeRTOS static allocation support - provide idle task stack/TCB
  */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
  * @brief  FreeRTOS static allocation support - provide timer task stack/TCB
  */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif /* USE_FREERTOS */

/******************************************************************************/
/*                 STM32F7xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f7xx.s).                                               */
/******************************************************************************/



/**
  * @brief  This function handles External line 1 interrupt request.
  * @param  None
  * @retval None
  *
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(WAKEUP_BUTTON_PIN);
}*/

/**
  * @brief  This function handles External line 2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
  * @brief  This function handles External line 15_10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  /* Interrupt handler shared between DIO8 (LoRa), SD_DETECT pin, USER_KEY button and touch screen interrupt */
#ifdef ENABLE_LR20XX
  if (__HAL_GPIO_EXTI_GET_IT(DIO8_PIN) != RESET)
  {
    HAL_GPIO_EXTI_IRQHandler(DIO8_PIN);
  }
  else
#endif
  if (__HAL_GPIO_EXTI_GET_IT(SD_DETECT_PIN) != RESET)
  {
    HAL_GPIO_EXTI_IRQHandler(SD_DETECT_PIN | TS_INT_PIN | AUDIO_IN_INT_GPIO_PIN);   /* SD detect event or touch screen interrupt */
  }
  else
  {     /* User button event or Touch screen interrupt */
    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
  }
}

/**
  * @brief This function handles DMA2 Stream 7 interrupt request.
  * @param None
  * @retval None
  */
void AUDIO_IN_SAIx_DMAx_IRQHandler(void)
{
    //if (haudio_in_sai.hdmarx)
        HAL_DMA_IRQHandler(haudio_in_sai.hdmarx);

    //HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

/**
  * @brief  Handles SDRAM DMA transfer interrupt request.
  * @retval None
  */
void BSP_SDRAM_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(sdramHandle.hdma); 
}

/**
  * @brief  DMA interrupt handler.
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{
  BSP_CAMERA_DMA_IRQHandler();
}

/**
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void AUDIO_OUT_SAIx_DMAx_IRQHandler(void)
{
  HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

/**
  * @brief  DCMI interrupt handler.
  * @param  None
  * @retval None
  */
void DCMI_IRQHandler(void)
{
  BSP_CAMERA_IRQHandler();
}

/**
  * @brief  This function handles DMA2D Handler.
  * @param  None
  * @retval None
  */
void DMA2D_IRQHandler(void)
{
  //BSP_LCD_DMA2D_IRQHandler();
  for (;;) asm("nop");
}

#ifdef ENABLE_LR20XX
/* Flag set by DIO8 interrupt - checked by LR20xx_service() for faster response */
volatile uint8_t dio8_irq_pending = 0;
#endif /* ENABLE_LR20XX */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
