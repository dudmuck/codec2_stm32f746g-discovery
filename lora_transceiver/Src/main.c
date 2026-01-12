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
#include "lr20xx_radio_fifo.h"
#ifdef USE_FREERTOS
#include "freertos_tasks.h"
#endif

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
uint8_t frames_per_sec;
unsigned nsamp;
unsigned nsamp_x2;
uint16_t _bytes_per_frame;
uint8_t frame_length_bytes;
uint8_t lora_payload_length;
uint8_t sf_at_500KHz;
uint16_t lora_bw_khz;
unsigned inter_pkt_timeout;

/* SF adjustment for different bandwidths.
 * To maintain same data rate: sf_new = sf_500kHz - log2(500/BW_kHz)
 * 500kHz: adjustment = 0
 * 250kHz: adjustment = 1
 * 125kHz: adjustment = 2
 */
#ifndef LORA_BW_KHZ
#define LORA_BW_KHZ 500
#endif

#if LORA_BW_KHZ >= 500
#define SF_BW_ADJUSTMENT 0
#elif LORA_BW_KHZ >= 250
#define SF_BW_ADJUSTMENT 1
#elif LORA_BW_KHZ >= 125
#define SF_BW_ADJUSTMENT 2
#elif LORA_BW_KHZ >= 62
#define SF_BW_ADJUSTMENT 3
#else
#define SF_BW_ADJUSTMENT 4
#endif

volatile int rx_size;
volatile float rx_rssi;
volatile float rx_snr;
#ifdef ENABLE_LR20XX
volatile uint16_t rx_decode_idx;    /* how many bytes decoded from streaming RX */
#endif

volatile uint32_t txStartAt;
//yyy volatile uint32_t cycleStartAt;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

loraAppHal_t appHal;

/* Private functions ---------------------------------------------------------*/

volatile uint32_t uart_irq_count = 0;
volatile uint32_t uart_error_flags = 0;
volatile uint32_t uart_instance_corrupted = 0;

void USART1_IRQHandler()
{
    uart_irq_count++;

    /* Check for corruption before accessing Instance */
    if (UartHandle.Instance != USART1) {
        uart_instance_corrupted = (uint32_t)UartHandle.Instance;
        /* Disable this IRQ to prevent infinite loop */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        return;
    }

    /* Read and clear any error flags before processing - prevents infinite IRQ loop */
    uint32_t isrflags = READ_REG(UartHandle.Instance->ISR);
    uart_error_flags = isrflags;  /* Save for debugging */

    if (isrflags & USART_ISR_ORE)  /* Overrun error */
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_CLEAR_OREF);
    if (isrflags & USART_ISR_FE)   /* Framing error */
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_CLEAR_FEF);
    if (isrflags & USART_ISR_NE)   /* Noise error */
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_CLEAR_NEF);
    if (isrflags & USART_ISR_PE)   /* Parity error */
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_CLEAR_PEF);

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
            /* Two columns: 0-4 in first, 5-8 in second */
            if (tx > (BPS_SELECT_BASE_X + BPS_SELECT_STEP_X)) {
                selection_state += 5;
            }
            if (selection_state > 8)
                selection_state = 8;
        }
    }

    BSP_LCD_SetFont(&Font24);
    while (modeStr[i] != NULL) {
        if (selection_state == i) {
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
            _y = BPS_SELECT_BASE_Y;
            _x += BPS_SELECT_STEP_X;    /* next column */
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
    /* Priority 10 = lower than audio (5) so UART TX doesn't starve encoder callbacks */
    HAL_NVIC_SetPriority(DISCOVERY_COM1_IRQn, 10, 0);
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

    /* Initialize audio output early to prevent floating speaker noise during rate selection.
     * Will be reinitialized with correct rate in AudioLoopback_demo() */
    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 0, I2S_AUDIOFREQ_16K);
    BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);

    while (ow == NULL) {
        uint16_t x, y;
        /* Check UART for rate selection: '0'-'5' or '8' */
        if (uartReceived) {
            uint8_t handled = 0;
            uartReceived = 0;
            HAL_UART_Receive_IT(&UartHandle, &rxchar, 1);
            printf("rxchar '%c'\r\n", rxchar);
            if (rxchar >= '0' && rxchar <= '8') {
                int8_t rate_idx = rxchar - '0';
                pressed_bitrate = rate_idx;
                printf("UART selected rate: %s\r\n", modeStr[rate_idx]);
                handled = 1;
            }
            if (!handled) {
                printf("Select Opus rate:\r\n");
                printf("  0: 6K   1: 8K   2: 12K  3: 16K  4: 24K\r\n");
                printf("  5: 32K  6: 48K  7: 64K  8: 96K\r\n");
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
                if (pressed_bitrate != -1) {
                    /* Set LCD Foreground Layer  */
                    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
                    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
                    /* Clear the LCD */
                    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
                    BSP_LCD_Clear(LCD_COLOR_WHITE);
                    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
                    /* Use 60ms frames for 64K and 96K modes to allow multi-packet TX */
                    if (pressed_bitrate == OPUS_WRAPPER_MODE_64K ||
                        pressed_bitrate == OPUS_WRAPPER_MODE_96K) {
                        ow = opus_wrapper_create_ex(pressed_bitrate, 60);
                    } else {
                        ow = opus_wrapper_create(pressed_bitrate);
                    }
                    if (ow == NULL) {
                        printf("ERROR: opus_wrapper_create(%d) failed\r\n", pressed_bitrate);
                        BSP_LCD_DisplayStringAt(20, 10, (uint8_t *)"opus_wrapper_create() failed", CENTER_MODE);
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
    } // ..while (ow == NULL)

    /* Clear the LCD */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

    {
        char buf[32];
        const char* str;
        /* Opus at 8kHz, 40ms frames = 320 samples per frame
         * Frame sizes are VBR but approximate:
         *   6K:  ~30 bytes/frame     12K: ~60 bytes/frame
         *   8K:  ~40 bytes/frame     16K: ~80 bytes/frame
         *   24K: ~120 bytes/frame    32K: ~160 bytes/frame
         *   48K: ~240 bytes/frame    64K: ~320 bytes/frame
         *   96K: ~480 bytes/frame
         * At 25 fps (40ms frames), we need to stream ~payload_bytes every 40ms
         * LR2021 max: ~100kbps at 1000kHz BW, SF5
         */
        /* Maximize frames per packet (max LoRa payload = 255 bytes).
         * More frames per packet = fewer packet boundaries = smoother audio.
         * 64K/96K modes have frames > 255 bytes, not supported over LoRa. */
        switch (selected_bitrate) {
            case OPUS_WRAPPER_MODE_6K:
                lora_payload_length = 240;  // 8 frames @ 30 bytes each
                sf_at_500KHz = 10;
                lora_bw_khz = 500;
                str = "6K";
                break;
            case OPUS_WRAPPER_MODE_8K:
                lora_payload_length = 240;  // 6 frames @ 40 bytes each
                sf_at_500KHz = 10;
                lora_bw_khz = 500;
                str = "8K";
                break;
            case OPUS_WRAPPER_MODE_12K:
                lora_payload_length = 240;  // 4 frames @ 60 bytes each
                sf_at_500KHz = 11;
                lora_bw_khz = 500;
                str = "12K";
                break;
            case OPUS_WRAPPER_MODE_16K:
                lora_payload_length = 240;  // 3 frames @ 80 bytes each
                sf_at_500KHz = 11;
                lora_bw_khz = 500;
                str = "16K";
                break;
            case OPUS_WRAPPER_MODE_24K:
                lora_payload_length = 240;  // 2 frames @ 120 bytes each
                sf_at_500KHz = 12;
                lora_bw_khz = 500;
                str = "24K";
                break;
            case OPUS_WRAPPER_MODE_32K:
                lora_payload_length = 160;  // 1 frame @ 160 bytes
                sf_at_500KHz = 9;           // need faster SF for higher rate
                lora_bw_khz = 500;
                str = "32K";
                break;
            case OPUS_WRAPPER_MODE_48K:
                lora_payload_length = 240;  // 1 frame @ 240 bytes
                sf_at_500KHz = 8;
                lora_bw_khz = 812;          // higher BW for faster TX
                str = "48K";
                break;
            case OPUS_WRAPPER_MODE_64K:
                /* 60ms frame @ 64kbps = 480 bytes/frame, need 2 packets */
                lora_payload_length = 240;  // 2 packets: 240 + 240 bytes
                sf_at_500KHz = 6;
                lora_bw_khz = 1000;         // highest BW for 64K
                str = "64K";
                break;
            case OPUS_WRAPPER_MODE_96K:
                /* 60ms frame @ 96kbps = 720 bytes/frame, need 3 packets */
                lora_payload_length = 240;  // 3 packets: 240 + 240 + 240 bytes
                sf_at_500KHz = 5;
                lora_bw_khz = 1000;         // highest BW for 96K
                str = "96K";
                break;
            default: str = NULL;
        } // ..switch (selected_bitrate)

        /* Apply SF adjustment for bandwidth.
         * Lower bandwidths need lower SF to maintain same data rate.
         * Minimum SF is 5 for LoRa. */
        if (SF_BW_ADJUSTMENT > 0) {
            int new_sf = sf_at_500KHz - SF_BW_ADJUSTMENT;
            sf_at_500KHz = (new_sf >= 5) ? new_sf : 5;
        }

        sprintf(buf, "%c  %s  %c", micLeftEn ? 'L' : ' ', str, micRightEn ? 'R' : ' ');
        BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)buf, CENTER_MODE);
    }
    nsamp = opus_wrapper_samples_per_frame(ow);
    nsamp_x2 = nsamp * 2;
    /* frames_per_sec = sample_rate / samples_per_frame = 1000 / frame_ms */
    frames_per_sec = 1000 / opus_wrapper_get_frame_ms(ow);  /* 25 fps for 40ms, ~17 fps for 60ms */

    /* Calculate _bytes_per_frame BEFORE start_radio() so FSK can use it.
     * Opus uses VBR, so _bytes_per_frame is approximate based on bitrate. */
    {
        int bitrate = opus_wrapper_get_bitrate(ow);
        int frame_ms = opus_wrapper_get_frame_ms(ow);
        int mode = opus_wrapper_get_mode(ow);
        printf("DEBUG: mode=%d bitrate=%d frame_ms=%d\r\n", mode, bitrate, frame_ms);
        /* bytes_per_frame = (bitrate * frame_duration_ms) / (8 * 1000)
         * For 40ms frame: bytes = bitrate * 40 / 8000 = bitrate / 200
         * For 60ms frame: bytes = bitrate * 60 / 8000 = bitrate / 133 */
        _bytes_per_frame = (bitrate * frame_ms) / 8000;
    }

    /* radio is started after bw/sf and _bytes_per_frame are known */
    start_radio();

    printf("nsamp:%u, _bytes_per_frame:%u, frame_length_bytes:%u\r\n", nsamp, _bytes_per_frame, frame_length_bytes);

    print_streaming_timing_analysis();

    /* Auto-adjust SF if margin is insufficient for reliable streaming */
    adjust_sf_for_streaming();

    /* Calculate streaming configuration based on (possibly adjusted) SF */
    calculate_streaming_config();

    /* Multi-packet modes (64K, 96K) need faster encoding because we can't
     * pipeline encode with TX. Reduce complexity to 2 (from default 5).
     * Testing shows: 0-1 same, 2 only slow during audio, 3+ causes defects. */
    if (streaming_cfg.packets_per_frame > 1) {
        opus_wrapper_set_complexity(ow, 2);
        printf("Multi-packet mode: reduced Opus complexity to %d\r\n",
               opus_wrapper_get_complexity(ow));
    }

    /* Configure TX FIFO IRQ: alert when FIFO level drops below threshold (room for more data)
     * or on underflow. Threshold set to _bytes_per_frame so we get IRQ when there's room
     * for another Opus frame.
     * Configure RX FIFO IRQ: alert when FIFO level exceeds threshold (data available to read)
     * or on overflow.
     * Note: LR2021 FIFO is only 256 bytes. For multi-packet mode (64K, 96K), we need to drain
     * the FIFO frequently to avoid overflow since packets arrive back-to-back. Use 64 bytes
     * as threshold to ensure we drain well before the 256-byte limit. */
    {
        uint16_t rx_threshold, tx_threshold;
        if (streaming_cfg.packets_per_frame > 1) {
            /* Multi-packet mode: threshold at 64 bytes.
             * With single-read FIFO callback (no loop), more frequent smaller
             * reads are faster than fewer large reads that risk overflow.
             * Headroom: 256-64=192 bytes before overflow at 95 kbps. */
            rx_threshold = 64;
            tx_threshold = 64;
        } else {
            /* Normal mode: threshold at frame size for efficiency */
            rx_threshold = _bytes_per_frame;
            tx_threshold = _bytes_per_frame;
            if (rx_threshold > 200) rx_threshold = 200;  /* Leave headroom below 256 */
            if (tx_threshold > 200) tx_threshold = 200;
        }
        lr20xx_radio_fifo_cfg_irq(NULL,
                                  LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH | LR20XX_RADIO_FIFO_FLAG_OVERFLOW,  /* rx_fifo_irq_enable */
                                  LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW | LR20XX_RADIO_FIFO_FLAG_UNDERFLOW,  /* tx_fifo_irq_enable */
                                  rx_threshold,               /* rx_fifo_high_threshold */
                                  tx_threshold,               /* tx_fifo_low_threshold */
                                  0,                          /* rx_fifo_low_threshold */
                                  0);                         /* tx_fifo_high_threshold */
    }

#ifdef USE_FREERTOS
    /* Initialize FreeRTOS tasks for concurrent encode/TX.
     * Tasks will start when scheduler begins. */
    freertos_tasks_init();
    printf("FreeRTOS initialized, starting scheduler...\r\n");
    /* Start scheduler - this never returns */
    freertos_start();
    /* Should never reach here */
    printf("\e[31mFreeRTOS scheduler returned!\e[0m\r\n");
#else
    AudioLoopback_demo();
#endif

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
    else {
        /* Use integer formatting to avoid heap-allocating floating point printf */
        int rssi_int = (int)rssi;
        int rssi_frac = (int)((rssi - rssi_int) * 10) % 10;
        if (rssi_frac < 0) rssi_frac = -rssi_frac;
        int snr_int = (int)snr;
        int snr_frac = (int)((snr - snr_int) * 10) % 10;
        if (snr_frac < 0) snr_frac = -snr_frac;
        sprintf(str, "%d.%ddBm %d.%ddB", rssi_int, rssi_frac, snr_int, snr_frac);
    }

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
