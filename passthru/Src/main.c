/**
  ******************************************************************************
  * @file    BSP/Src/main.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the STM32746G Discovery BSP Drivers
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global extern variables ---------------------------------------------------*/
uint8_t NbLoop = 1;
#ifndef USE_FULL_ASSERT
uint32_t    ErrorCounter = 0;
#endif

volatile uint8_t uartReceived;
uint8_t rxchar;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);



/* Private functions ---------------------------------------------------------*/

void USART1_IRQHandler()
{
    HAL_UART_IRQHandler(&UartHandle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uartReceived = 1;
}

const unsigned audioRates[] = 
{
    I2S_AUDIOFREQ_192K,
    I2S_AUDIOFREQ_96K,
    I2S_AUDIOFREQ_48K,
    I2S_AUDIOFREQ_44K,
    I2S_AUDIOFREQ_32K,
    I2S_AUDIOFREQ_22K,
    I2S_AUDIOFREQ_16K,
    I2S_AUDIOFREQ_11K,
    I2S_AUDIOFREQ_8K,
    0
};
int audioRate_idx;
int pressed_audioRate;
bool pressed_8kresamp;
bool resamp8k;
uint8_t resamp_ratio;

const unsigned SELECT_BASE_Y = 50;
const unsigned SELECT_STEP_Y = 28;

const unsigned SELECT_BASE_X = 20;
const unsigned SELECT_STEP_X = 140;

static void Touchscreen_DrawBackground_resamp (bool pressed, uint16_t tx, uint16_t ty)
{
    if (pressed)
        pressed_8kresamp = ty >= BSP_LCD_GetYSize()-24;

    if (resamp8k) {
        if (pressed_8kresamp && pressed)
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
        else
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    } else {
        if (pressed_8kresamp && pressed)
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
        else
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    }

    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-24, (uint8_t *)"8k resample", RIGHT_MODE);
}

static void
Touchscreen_DrawBackground (uint8_t state)
{
    unsigned y = SELECT_BASE_Y , i = 0;
    unsigned x = SELECT_BASE_X;
    char str[16];

    BSP_LCD_SetFont(&Font24);
    while (audioRates[i] != 0) {
        if (state == i) {
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
            BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
            pressed_audioRate = i;
        } else {
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        }
        sprintf(str, "%u", audioRates[i]);
        BSP_LCD_DisplayStringAt(x, y, (uint8_t *)str, LEFT_MODE);
        y += SELECT_STEP_Y;
        if (i == 5) {
            y = SELECT_BASE_Y;
            x += SELECT_STEP_X;    // next column
        }
        i++;
    }

}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    uint8_t status;
    int state;
    uint8_t  lcd_status = LCD_OK;
    TS_StateTypeDef  TS_State;

    /* Enable the CPU Cache */
    CPU_CACHE_Enable();

    /* STM32F7xx HAL library initialization:
    - Configure the Flash prefetch, instruction and Data caches
    - Configure the Systick to generate an interrupt each 1 msec
    - Set NVIC Group Priority to 4
    - Global MSP (MCU Support Package) initialization
    */
    HAL_Init();
    /* Configure the system clock to 200 Mhz */
    SystemClock_Config();

    UartHandle.Init.BaudRate = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = UART_MODE_RX | UART_MODE_TX;
    BSP_COM_Init(COM1, &UartHandle);

    HAL_UART_Receive_IT(&UartHandle, &rxchar, 1);
    HAL_NVIC_SetPriority(DISCOVERY_COM1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DISCOVERY_COM1_IRQn);

    BSP_LED_Init(LED1);

    /* Configure the User Button in GPIO Mode */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

    /*##-1- Initialize the LCD #################################################*/
    /* Initialize the LCD */
    lcd_status = BSP_LCD_Init();
    ASSERT(lcd_status != LCD_OK);

    /* Initialize the LCD Layers */
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);

    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

    /* Clear the LCD */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_Clear(LCD_COLOR_WHITE);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
    BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"select audio rate", CENTER_MODE);

    /* Configure the User Button in GPIO Mode */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

    status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

    if (status != TS_OK)
    {
      BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)"Touchscreen cannot be initialized", CENTER_MODE);
    }
    else
    {
        state = -1;
        Touchscreen_DrawBackground(state);
        Touchscreen_DrawBackground_resamp (false, 0, 0);
    }

    pressed_audioRate = -1;
    for (audioRate_idx = -1; audioRate_idx < 0; ) {
        uint16_t x, y;
        int I;
        if (status == TS_OK) {
            BSP_TS_GetState(&TS_State);
            if (TS_State.touchDetected) {
                x = TS_State.touchX[0];
                y = TS_State.touchY[0];
                printf("touch x=%u, y=%u\r\n", x, y);
                if (y >= SELECT_BASE_Y) {
                    I = y - SELECT_BASE_Y;
                    I /= SELECT_STEP_Y;
                    if (x > (SELECT_BASE_X + SELECT_STEP_X)) {
                        I += 6;
                    }
                    printf("I:%d\r\n", I);
                    Touchscreen_DrawBackground(I);
                }
                Touchscreen_DrawBackground_resamp(true, x, y);
            } else if (pressed_audioRate != -1) {
                printf("release %d\r\n", pressed_audioRate);
                /* Set LCD Foreground Layer  */
                BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
                BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
                /* Clear the LCD */
                BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                BSP_LCD_Clear(LCD_COLOR_WHITE);
                BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

                audioRate_idx = pressed_audioRate;
            } else if (pressed_8kresamp) {
                resamp8k ^= true;
                printf("released_8k %s\r\n", resamp8k ? "on" : "off");
                pressed_8kresamp = false;
                Touchscreen_DrawBackground_resamp(false, 0, 0);
            }
        }
        HAL_Delay(10);
    } // ..while ()

    if (resamp8k) {
        resamp_ratio = audioRates[audioRate_idx] / 8000;
        printf("##### %u = %u / 8000 #####\r\n", resamp_ratio, audioRates[audioRate_idx]);
    }


    printf("using rate %u\r\n", audioRates[audioRate_idx]);
#if 0
#endif /* if 0 */
    AudioLoopback_demo(audioRates[audioRate_idx]);

} // ..main()

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  ASSERT(ret != HAL_OK);

  /* activate the OverDrive */
  ret = HAL_PWREx_ActivateOverDrive();
  ASSERT(ret != HAL_OK);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  ASSERT(ret != HAL_OK);
}

/**
  * @brief  Check for user input.
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint8_t CheckForUserInput(void)
{
  if (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    HAL_Delay(10);
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    return 1 ;
  }
  return 0;
}

/**
  * @brief EXTI line detection callbacks.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t debounce_time = 0;

  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    /* Prevent debounce effect for user key */
    if ((HAL_GetTick() - debounce_time) > 50)
    {
      debounce_time = HAL_GetTick();
    }
  }
  else if (GPIO_Pin == AUDIO_IN_INT_GPIO_PIN)
  {
    /* Audio IN interrupt */
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
