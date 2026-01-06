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

/* Global extern variables ---------------------------------------------------*/
uint8_t NbLoop = 1;
#ifndef USE_FULL_ASSERT
uint32_t    ErrorCounter = 0;
#endif

volatile uint8_t uartReceived;
uint8_t rxchar;

struct OPUS_WRAPPER *ow;
unsigned nsamp;
unsigned nsamp_x2;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Display_DemoDescription(void);
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

void rate_choice_SetHint()
{
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set Touchscreen Demo description */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 28);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)"select vocoder rate", CENTER_MODE);
    
}

const char * const modeStr[] = {
    /* 0 */ "6K",
    /* 1 */ "8K",
    /* 2 */ "12K",
    /* 3 */ "16K",
    /* 4 */ "24K",
    /* 5 */ "32K",
    /* 6 */ "48K",
    /* 7 */ "64K",
    /* 8 */ "96K",
    NULL
};

const unsigned SELECT_BASE_Y = 50;
const unsigned BPS_SELECT_STEP_Y = 35;

const unsigned BPS_SELECT_BASE_X = 20;
const unsigned BPS_SELECT_STEP_X = 100;
int8_t _selected_bitrate;  // upon touch release
int8_t pressed_bitrate = -1;

int8_t selected_audiorate = 2;  // default 16k
int8_t pressed_audiorate = -1;

#define AUDIO_RATE_SELECT_BASE_X        230
#define AUDIO_RATE_SELECT_STEP_Y        32

#ifndef MIC_DISABLE
static bool pressed_micRight;
static bool pressed_micLeft;
#endif /* !MIC_DISABLE */
bool micRightEn;
bool micLeftEn;

int vocoder_rate_state;
int audio_rate_state;

#define MIC_LEFT_Y      (BSP_LCD_GetYSize() - 24)

static void
Touchscreen_DrawBackground(bool touched, uint16_t tx, uint16_t ty)
{
    unsigned _y = SELECT_BASE_Y , i = 0;
    unsigned _x = BPS_SELECT_BASE_X;

    if (touched) {
        if (tx < AUDIO_RATE_SELECT_BASE_X) {
            vocoder_rate_state = ty - SELECT_BASE_Y;
            vocoder_rate_state /= BPS_SELECT_STEP_Y;
            /* Two columns: 0-4 in first, 5-8 in second */
            if (tx > (BPS_SELECT_BASE_X + BPS_SELECT_STEP_X)) {
                vocoder_rate_state += 5;
            }
            if (vocoder_rate_state > 8)
                vocoder_rate_state = 8;
            printf("vocoder_rate_state:%d\r\n", vocoder_rate_state);
        } else {
            /* audio rate touch selection */
            audio_rate_state = ty - SELECT_BASE_Y;
            audio_rate_state /= AUDIO_RATE_SELECT_STEP_Y;
            printf("audio_rate_state:%d\r\n", audio_rate_state);
        }
    }

    BSP_LCD_SetFont(&Font24);
    while (modeStr[i] != NULL) {
        if (vocoder_rate_state == i) {
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
            BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
            pressed_bitrate = i;
        } else {
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        }
        BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)modeStr[i], LEFT_MODE);
        _y += BPS_SELECT_STEP_Y;
        if (i == 4) {
            _y = SELECT_BASE_Y;
            _x += BPS_SELECT_STEP_X;    /* next column */
        }
        i++;
    }

    /* audio rate: */
    _y = SELECT_BASE_Y;
    _x = AUDIO_RATE_SELECT_BASE_X;
    for (i = 0; i < 4; i++) {
        const char *str;
        switch (i) {
            case 0: str = "48k"; break;
            case 1: str = "32k"; break;
            case 2: str = "16k"; break;
            case 3: str = "8k"; break;
        }

        if (i == audio_rate_state) {
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            pressed_audiorate = i;
        } else {
            if (i == selected_audiorate)
                BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
            else
                BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        }

        if (i == selected_audiorate)
            BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
        else
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

        BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)str, LEFT_MODE);
        _y += AUDIO_RATE_SELECT_STEP_Y;
    }

#ifndef MIC_DISABLE
    /* micEn X base */
    _x = BSP_LCD_GetXSize()-135;

    /* microphone right enable */
    _y = BSP_LCD_GetYSize() - 52;
    if (micRightEn)
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    else
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    if (touched) {
        if (ty < MIC_LEFT_Y && ty > _y && tx >= _x) {
            pressed_micRight = true;
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
        }
    } else
        pressed_micRight = false;

    if (micRightEn)
        BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    else
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

    BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)"micRight", LEFT_MODE);

    /* microphone left enable */
    _y = MIC_LEFT_Y;
    if (micLeftEn)
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    else
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    if (touched) {
        if (ty >= MIC_LEFT_Y && tx >= _x) {
            pressed_micLeft = true;
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
        }
    } else {
        pressed_micLeft = false;
    }
    if (micLeftEn)
        BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    else
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

    BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)"micLeft", LEFT_MODE);
#endif /* !MIC_DISABLE */
} // ..Touchscreen_DrawBackground()

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    TS_StateTypeDef  TS_State;
    uint8_t status, lcd_status = LCD_OK;

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

    //HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
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

    Display_DemoDescription();

    rate_choice_SetHint();

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
#ifdef MIC_DISABLE
    #if (MIC_DISABLE==MIC_LEFT)
        micRightEn = true;
        micLeftEn = false;
    #elif(MIC_DISABLE==MIC_RIGHT)
        micRightEn = false;
        micLeftEn = true;
    #endif
#else
        micRightEn = true;  // default mic enabled
        micLeftEn = true;   // default mic enabled
#endif /* !MIC_DISABLE */
        vocoder_rate_state = -1;
        audio_rate_state = -1;
        Touchscreen_DrawBackground(false, 0, 0);
    }
    
    printf("Select vocoder: 0-8 for 6K/8K/12K/16K/24K/32K/48K/64K/96K\r\n");

    while (ow == NULL) {
        uint16_t x, y;

        /* Check for UART selection */
        if (uartReceived) {
            uartReceived = 0;
            HAL_UART_Receive_IT(&UartHandle, &rxchar, 1);
            if (rxchar >= '0' && rxchar <= '8') {
                pressed_bitrate = rxchar - '0';
                printf("UART selected mode %d (%s)\r\n", pressed_bitrate, modeStr[pressed_bitrate]);
            }
        }

        if (status == TS_OK) {
            BSP_TS_GetState(&TS_State);
            if (TS_State.touchDetected) {
                x = TS_State.touchX[0];
                y = TS_State.touchY[0];
                printf("touch x=%u, y=%u\r\n", x, y);
                Touchscreen_DrawBackground(true, x, y);
            } else {
                if (pressed_bitrate != -1) { /* release on vocoder rate selection (touch or UART) */
                    /* Auto-select audio rate based on Opus mode */
                    int opus_sr = opus_wrapper_mode_sample_rate(pressed_bitrate);
                    switch (opus_sr) {
                        case 8000:  selected_audiorate = 3; break;  /* 8k */
                        case 16000: selected_audiorate = 2; break;  /* 16k */
                        case 48000: selected_audiorate = 0; break;  /* 48k */
                        default:    selected_audiorate = 2; break;  /* default 16k */
                    }
                    printf("Auto-selected audio rate %d for Opus mode %d (sr=%d)\r\n",
                           selected_audiorate, pressed_bitrate, opus_sr);
                    /* Set LCD Foreground Layer  */
                    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
                    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
                    /* Clear the LCD */
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);
                    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
                    ow = opus_wrapper_create(pressed_bitrate);
                    if (ow == NULL) {
                        BSP_LCD_DisplayStringAt(20, 10, (uint8_t *)"opus_wrapper_create() failed", CENTER_MODE);
                        pressed_bitrate = -1;
                        HAL_Delay(500);
                        vocoder_rate_state = -1;
                    } else
                        _selected_bitrate = pressed_bitrate;
                } else if (pressed_audiorate != -1) {   /* release on audio rate selection */
                    selected_audiorate = pressed_audiorate;
                    printf("selected_audiorate:%u\r\n", selected_audiorate);
                }
#ifndef MIC_DISABLE
                if (pressed_micRight) {
                    micRightEn ^= true;
                    printf("micRightEn:%d\r\n", micRightEn);
                    pressed_micRight = false;
                } else if (pressed_micLeft) {
                    micLeftEn ^= true;
                    printf("micLeftEn:%d\r\n", micLeftEn);
                    pressed_micLeft = false;
                }
#endif /* !MIC_DISABLE */
                Touchscreen_DrawBackground(false, 0, 0);
            } // ..if (!TS_State.touchDetected)
        } // ..if (status == TS_OK)
        HAL_Delay(10);
    } // ..while (ow == NULL)

    /* Clear the LCD */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

    {
        char buf[48];
        const char* str;
        switch (_selected_bitrate) {
            case OPUS_WRAPPER_MODE_6K: str = "6K"; break;
            case OPUS_WRAPPER_MODE_8K: str = "8K"; break;
            case OPUS_WRAPPER_MODE_12K: str = "12K"; break;
            case OPUS_WRAPPER_MODE_16K: str = "16K"; break;
            case OPUS_WRAPPER_MODE_24K: str = "24K"; break;
            case OPUS_WRAPPER_MODE_32K: str = "32K"; break;
            case OPUS_WRAPPER_MODE_48K: str = "48K"; break;
            case OPUS_WRAPPER_MODE_64K: str = "64K"; break;
            case OPUS_WRAPPER_MODE_96K: str = "96K"; break;
            default: str = NULL;
        }
        sprintf(buf, "%c  %s  %c", micLeftEn ? 'L' : ' ', str, micRightEn ? 'R' : ' ');
        BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)buf, CENTER_MODE);
    }

    nsamp = opus_wrapper_samples_per_frame(ow);
    nsamp_x2 = nsamp * 2;
    printf("nsamp:%u\r\n", nsamp);

    AudioLoopback_demo(selected_audiorate);

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
  * @brief  Display main demo messages.
  * @param  None
  * @retval None
  */
static void Display_DemoDescription(void)
{
    uint8_t desc[50];

    /* Set LCD Foreground Layer  */
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

    /* Clear the LCD */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_Clear(LCD_COLOR_WHITE);

    /* Set the LCD Text Color */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

    /* Display LCD messages */
    BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G BSP", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Drivers examples", CENTER_MODE);

    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"Copyright (c) STMicroelectronics 2015", CENTER_MODE);

    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 60);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"Press User Button to start :", CENTER_MODE);
    sprintf((char *)desc, "AUDIO LOOPBACK example");
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 45, (uint8_t *)desc, CENTER_MODE);
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
        return 1;
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
