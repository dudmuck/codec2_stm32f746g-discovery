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
#include "radio.h"

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

struct CODEC2 *c2;
uint8_t frames_per_sec;
unsigned nsamp;
unsigned nsamp_x2;
uint8_t _bytes_per_frame;
uint8_t frame_length_bytes;
uint8_t lora_payload_length;
uint8_t sf_at_500KHz;
unsigned inter_pkt_timeout;

volatile int rx_size;
volatile float rx_rssi;
volatile float rx_snr;

volatile uint32_t txStartAt;
//yyy volatile uint32_t cycleStartAt;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

loraAppHal_t appHal;

/* Private functions ---------------------------------------------------------*/

void USART1_IRQHandler()
{
    HAL_UART_IRQHandler(&UartHandle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uartReceived = 1;
}

volatile unsigned Cnt;
/*extern volatile uint8_t uartTXing;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    Cnt = UartHandle->hdmatx->Instance->NDTR;
    if (Cnt == 0) {
        uartTXing = 0;
    } else {
        for (;;) asm("nop");
    }
}*/

void rate_choice_SetHint()
{
    /* Set LCD Foreground Layer  */
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

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
    /* 0 */ "3200",
    /* 1 */ "2400",
    /* 2 */ "1600",
    /* 3 */ "1400",
    /* 4 */ "1300",
    /* 5 */ "1200",

    /* 6 */ "",
    /* 7 */ "",
    /* 8 */ "700C",
    NULL
};

const unsigned BPS_SELECT_BASE_Y = 50;
const unsigned BPS_SELECT_STEP_Y = 35;

const unsigned BPS_SELECT_BASE_X = 20;
const unsigned BPS_SELECT_STEP_X = 100;
int8_t selected_bitrate;  // upon touch release
int8_t pressed_bitrate = -1;
int selection_state;

#ifndef MIC_DISABLE
static bool pressed_micRight;
static bool pressed_micLeft;
#endif /* !MIC_DISABLE */
bool micRightEn;
bool micLeftEn;

static void
Touchscreen_DrawBackground(bool touched, uint16_t tx, uint16_t ty)
{
    unsigned mic_x_base = BSP_LCD_GetXSize() - 60;
    unsigned _y = BPS_SELECT_BASE_Y, i = 0;
    unsigned _x = BPS_SELECT_BASE_X;

    if (touched && tx < (mic_x_base-20)) {
        if (ty >= BPS_SELECT_BASE_Y) {
            selection_state = ty - BPS_SELECT_BASE_Y;
            selection_state /= BPS_SELECT_STEP_Y;
            if (tx > (BPS_SELECT_BASE_X + BPS_SELECT_STEP_X)) {
                selection_state += 6;
            }
        }
    }

    BSP_LCD_SetFont(&Font24);
    while (modeStr[i] != NULL) {
        if (selection_state == i) {
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
            BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
            if (modeStr[i][0] != 0) // zero-length modes dont exist
                pressed_bitrate = i;
        } else {
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        }
        BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)modeStr[i], LEFT_MODE);
        _y += BPS_SELECT_STEP_Y;
        if (i == 5) {
            _y = BPS_SELECT_BASE_Y;
            _x += BPS_SELECT_STEP_X;    // next column
        }
        i++;
    }

#ifndef MIC_DISABLE
    /////////////////////////
    BSP_LCD_SetFont(&Font12);
    _x = mic_x_base;
    _y = BSP_LCD_GetYSize() / 3;
    BSP_LCD_SetTextColor(micRightEn ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    if (touched) {
        if (ty >= _y && ty <= _y+12 && tx >= _x) {
            pressed_micRight = true;
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
        }
    } else
        pressed_micRight = false;

    BSP_LCD_SetBackColor(micRightEn ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)"micRight", LEFT_MODE);

    /////////////////////////
    _y = _y * 2;
    BSP_LCD_SetTextColor(micLeftEn ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    if (touched) {
        if (ty >= _y && ty <= _y+12 && tx >= _x) {
            pressed_micLeft = true;
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
        }
    } else
        pressed_micLeft = false;

    BSP_LCD_SetBackColor(micLeftEn ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(_x, _y, (uint8_t *)" micLeft", LEFT_MODE);
#endif /* !MIC_DISABLE */
}

#define USARTx_DMA_TX_IRQn                DMA2_Stream7_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream2_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream7_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream2_IRQHandler


#define USARTx_TX_DMA_STREAM              DMA2_Stream7
#define USARTx_RX_DMA_STREAM              DMA2_Stream2
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_4

#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
/*void USARTx_DMA_TX_IRQHandler(void)
{ 
    HAL_DMA_IRQHandler(UartHandle.hdmatx);
}*/

//extern volatile uint8_t uartTXing;
/*void dma_tx_complete(DMA_HandleTypeDef *_hdma)
{
    uartTXing = 0;
}*/

/* COM1 = USART1 */
#if 0
void UART_DMA_Init(COM_TypeDef COM)
{
    static DMA_HandleTypeDef hdma_tx;
    
    DMAx_CLK_ENABLE();

    hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
    hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);

    /* NVIC configuration for DMA transfer complete interrupt (USART6_TX) */
    HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
    //HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);

    /* NVIC for USART, to catch the TX complete */
    //HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
    //HAL_NVIC_EnableIRQ(USARTx_IRQn);

    //HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma))
    //HAL_DMA_RegisterCallback(&hdma_tx, HAL_DMA_XFER_CPLT_CB_ID, dma_tx_complete);
}
#endif /* if 0 */


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

    //UART_DMA_Init(COM1);
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
        selection_state = -1;
        Touchscreen_DrawBackground(false, 0, 0);
    }

    while (c2 == NULL) {
        uint16_t x, y;
        if (status == TS_OK) {
            BSP_TS_GetState(&TS_State);
            if (TS_State.touchDetected) {
                x = TS_State.touchX[0];
                y = TS_State.touchY[0];
                printf("touch x=%u, y=%u\r\n", x, y);
                Touchscreen_DrawBackground(true, x, y);
            } else {
                if (pressed_bitrate != -1) {
                    /* Set LCD Foreground Layer  */
                    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
                    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
                    /* Clear the LCD */
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);
                    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
                    c2 = codec2_create(pressed_bitrate);
                    if (c2 == NULL) {
                        BSP_LCD_DisplayStringAt(20, 10, (uint8_t *)"codec2_create() failed", CENTER_MODE);
                        pressed_bitrate = -1;
                        HAL_Delay(500);
                        selection_state = -1;
                    } else
                        selected_bitrate = pressed_bitrate;
                } // ..if (pressed_bitrate != -1)
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
    } // ..while (c2 == NULL)

    /* Clear the LCD */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

    {
        char buf[32];
        const char* str;
        switch (selected_bitrate) {
            case CODEC2_MODE_3200:
                lora_payload_length = 128;
                sf_at_500KHz = 9;   // at 99% duty at sf10
                inter_pkt_timeout = 40;
                str = "3200";
                break;
            case CODEC2_MODE_2400:
                lora_payload_length = 96;
                sf_at_500KHz = 10;
                inter_pkt_timeout = 40;
                str = "2400";
                break;
            case CODEC2_MODE_1600:
                lora_payload_length = 128;
                sf_at_500KHz = 11;
                inter_pkt_timeout = 40;
                str = "1600";
                break;
            case CODEC2_MODE_1400:
                lora_payload_length = 112;
                sf_at_500KHz = 11;
                inter_pkt_timeout = 40;
                str = "1400";
                break;
            case CODEC2_MODE_1300:
                lora_payload_length = 52;
                sf_at_500KHz = 11;
                inter_pkt_timeout = 40;
                str = "1300";
                break;
            case CODEC2_MODE_1200:
                lora_payload_length = 96;
                sf_at_500KHz = 11;
                inter_pkt_timeout = 40;
                str = "1200";
                break;
            case CODEC2_MODE_700C:
                lora_payload_length = 56;
                sf_at_500KHz = 12;
                inter_pkt_timeout = 40;
                str = "700C";
                break;
            default: str = NULL;
        } // ..switch (selected_bitrate)

        sprintf(buf, "%c  %s  %c", micLeftEn ? 'L' : ' ', str, micRightEn ? 'R' : ' ');
        BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)buf, CENTER_MODE);
    }
    nsamp = codec2_samples_per_frame(c2);
    nsamp_x2 = nsamp * 2;
    frames_per_sec = 8000 / nsamp;

    /* radio is started after bw/sf is known */
    start_radio();

    if (selected_bitrate == CODEC2_MODE_1300 || selected_bitrate == CODEC2_MODE_700C) {
        _bytes_per_frame = codec2_bits_per_frame(c2) / 4;       // dual frame for integer number of bytes
        frame_length_bytes = _bytes_per_frame >> 1;
    } else {
        _bytes_per_frame = codec2_bits_per_frame(c2) / 8;
    }

    printf("nsamp:%u, _bytes_per_frame:%u, frame_length_bytes:%u\r\n", nsamp, _bytes_per_frame, frame_length_bytes);
    AudioLoopback_demo();

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
    } else if (GPIO_Pin == lorahal.irq_pin) {
        if (lorahal.irqTopHalf) {
            lorahal.irqTopHalf();
        }
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

void lcd_print_tx_duration(int dur, unsigned interval)
{
    char str[24];
    float duty = (float)dur / interval;

    sprintf(str, "%02u%%  %3d / %u   ", (unsigned)(duty * 100), dur, interval);
    BSP_LCD_SetFont(&Font24);
    if (dur <= 0) {
        BSP_LCD_SetBackColor(LCD_COLOR_RED);
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    } else {
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    }
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-24, (uint8_t *)str, LEFT_MODE);
}

void lcd_print_rssi_snr(float rssi, float snr)
{
    char str[24];
    if (rssi == 0 && snr == 0)
        sprintf(str, "                  ");
    else
        sprintf(str, "%.1fdBm %.1fdB", (double)rssi, (double)snr);

    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-24, (uint8_t *)str, LEFT_MODE);
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    rx_size = size;
    rx_rssi = rssi;
    rx_snr = snr;

    lcd_print_rssi_snr(rssi, snr);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
