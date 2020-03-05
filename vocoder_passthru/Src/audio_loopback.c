/**
  ******************************************************************************
  * @file    BSP/Src/audio_loopback.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the audio feature in the
  *          stm32746g_discovery driver
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
#include <stdio.h>
#include "string.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

#define N_ENCS     64
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;

unsigned audio_block_size;
unsigned navgs_;
float step;
uint8_t bytesPerFrame;

uint16_t *audio_buffer_in;
uint16_t *audio_buffer_in_B;
uint16_t *audio_buffer_out;
uint16_t *audio_buffer_out_B;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t  audio_rec_buffer_state;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t audio_in_vol = DEFAULT_AUDIO_IN_VOLUME;
uint8_t audio_out_vol = 64;

uint8_t encoded[N_ENCS][8];
int enc_in_idx, enc_out_idx;
uint8_t latency = N_ENCS / 2;   // initial default value assigned

typedef enum {
    PRESSED_NONE = 0,
    PRESSED_UP,
    PRESSED_DOWN
} pressed_e;

void svc_uart()
{
    const unsigned y_base = 30;
    TS_StateTypeDef  TS_State;
    static pressed_e pressed = PRESSED_NONE;
    int late, my_in = enc_in_idx;
    char str[32];
    if (my_in < enc_out_idx)
        my_in += N_ENCS;
    late = my_in - enc_out_idx;
    sprintf(str, "latency %d   ", late);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, y_base+26, (uint8_t *)str, LEFT_MODE);

    BSP_TS_GetState(&TS_State);

    if (TS_State.touchDetected) {
        uint16_t x, y;
        x = TS_State.touchX[0];
        y = TS_State.touchY[0];
        if (x < 50) {
            //printf("touch x=%u, y=%u ", x, y);
            if (y >= (y_base+52) && y <= (y_base+78)) {
                //printf("DOWNpressed ");
                pressed = PRESSED_DOWN;
            } else if (y >= y_base && y <= (y_base+26)) {
                //printf("UPpressed ");
                pressed = PRESSED_UP;
            }
        }
    } else {
        if (pressed == PRESSED_UP) {
            //printf("releaseUP ");
            if (++enc_in_idx == N_ENCS)
                enc_in_idx = 0;
        } else if (pressed == PRESSED_DOWN) {
            //printf("releaseDOWN ");
            if (enc_in_idx == 0)
                enc_in_idx = N_ENCS-1;
            else
                enc_in_idx--;
        }
        pressed = PRESSED_NONE;
    }

    if (pressed == PRESSED_UP) {
        BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    } else {
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    }
    BSP_LCD_DisplayStringAt(0, y_base, (uint8_t *)"UP", LEFT_MODE);
    if (pressed == PRESSED_DOWN) {
        BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    } else {
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    }
    BSP_LCD_DisplayStringAt(0, y_base+52, (uint8_t *)"DOWN", LEFT_MODE);

    /********************************************/
    if (uartReceived == 0)
        return;

    uartReceived = 0;
    HAL_UART_Receive_IT(&UartHandle, &rxchar, 1);

    printf("rxchar '%c'\r\n", rxchar);
    if (rxchar == 'q' || rxchar == 'a') {
        if (rxchar == 'q'  && audio_in_vol < 100) {
            audio_in_vol++;
        } else if (rxchar == 'a' && audio_in_vol > 0) {
            audio_in_vol--;
        }
        if (BSP_AUDIO_IN_SetVolume(audio_in_vol) == AUDIO_OK)
            printf("inVolOk%u\r\n", audio_in_vol);
        else
            printf("inVolFail%u\r\n", audio_in_vol);
    } else if (rxchar == 'w' || rxchar == 's') {
        if (rxchar == 'w' && audio_out_vol < 100)
            audio_out_vol++;
        if (rxchar == 's' && audio_out_vol > 0)
            audio_out_vol--;

        if (BSP_AUDIO_OUT_SetVolume(audio_out_vol) == AUDIO_OK)
            printf("outVolOk%u\r\n", audio_out_vol);
        else
            printf("outVolFail%u\r\n", audio_out_vol);
    }
}

#define N_HISTORY       12
void decimated_encode(const short *in)
{
    short to_encoder[320];
    unsigned i, x;
    int sum;
    int _in_idx;
    int num_avgs = navgs_;

    for (x = 0; x < nsamp; x++) {
        int avg;
        sum = 0;
        for (i = 0; i < navgs_; i++) {
#if 0
            sum += *in++;
            in++;   // skip other mic
#endif /* if 0 */
            int v = *in++;
            v += *in++; // taking average of both microphones
            sum += (v / 2);
        }
        avg = sum / num_avgs;
        to_encoder[x] = avg;
        /*if (x == 0)
            printf("MIC:%d = %d / %u\r\n", avg, sum, navgs_);*/
    }
    /*printf("abs:%u\r\n", audio_block_size);*/

    codec2_encode(c2, encoded[enc_in_idx], to_encoder);

    BSP_LCD_SetFont(&Font12);
    {   /* print encoded */
        char i, str[64], *ptr;
        unsigned n, y = BSP_LCD_GetYSize() - (N_HISTORY * 13);
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        _in_idx = enc_in_idx - N_HISTORY;
        if (_in_idx < 0)
            _in_idx += N_ENCS;
        for (i = 0; i < N_HISTORY; i++) {
            ptr = str;
            for (n = 0; n < bytesPerFrame; n++) {
                sprintf(ptr, "%02x ", encoded[_in_idx][n]);
                ptr += 3;
            }
            BSP_LCD_DisplayStringAt(0, y, (uint8_t *)str, LEFT_MODE);
            y += 13;
            if (++_in_idx == N_ENCS)
                _in_idx = 0;
        }
    }

    if (++enc_in_idx == N_ENCS)
        enc_in_idx = 0;
}


void c2_passthru(short *out, const short *in)
{
    static short from_decoderA[320];
    static short from_decoderB[320];
    static uint8_t fdb = 0;
    short *decBuf, *prevDecBuf;
    unsigned i, x;

    /* 32ksps to 8ksps */

    decimated_encode(in);

    decBuf = fdb ? from_decoderB : from_decoderA;
    prevDecBuf = fdb ? from_decoderA : from_decoderB;
    codec2_decode(c2, decBuf, encoded[enc_out_idx]);
    if (++enc_out_idx == N_ENCS)
        enc_out_idx = 0;

    for (x = 0; x < nsamp; x++) {
        short y0 = prevDecBuf[x];
        short dy;
        if (x < (nsamp-1))
            dy = (prevDecBuf[x+1] - y0) * step;
        else
            dy = (decBuf[0] - y0) * step;

        for (i = 0; i < navgs_; i++) {
            short v = y0 += dy;
            *out++ = v;
            *out++ = v;
        }
    }
    fdb ^= 1;
}

/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
void AudioLoopback_demo(uint8_t audioRateIdx)
{
    unsigned sec, cpy_cnt = 0;
    unsigned halfAt, fullAt;
    unsigned audioRate;

    switch (audioRateIdx) {
            case 0: audioRate = I2S_AUDIOFREQ_48K; break;
            case 1: audioRate = I2S_AUDIOFREQ_32K; break;
            case 2: audioRate = I2S_AUDIOFREQ_16K; break;
            case 3: audioRate = I2S_AUDIOFREQ_8K; break;
            default:
                BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                BSP_LCD_SetTextColor(LCD_COLOR_RED);
                BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"  bad audio rate idx", CENTER_MODE);
                BSP_LCD_DisplayStringAt(0, 60, (uint8_t *)" Try to reset board ", CENTER_MODE);
                for (;;) asm("nop");
                return;
    }

    BSP_LCD_SetFont(&Font12);
    /* Initialize Audio Recorder */
    if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, OUTPUT_DEVICE_HEADPHONE, audioRate, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) == AUDIO_OK)
    {
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"  AUDIO RECORD INIT OK  ", CENTER_MODE);
    }
    else
    {
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"  AUDIO RECORD INIT FAIL", CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, 60, (uint8_t *)" Try to reset board ", CENTER_MODE);
    }

    /* Display the state on the screen */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, 45, (uint8_t *)"Microphones sound streamed to headphones", CENTER_MODE);

    //audio_block_size = nsamp * 8;
    audio_block_size = nsamp * (audioRate / 8000) * 4;
    navgs_ = audioRate / 8000;
    step = 1.0 / navgs_;

    /* Initialize SDRAM buffers */
    audio_buffer_in = (uint16_t*)AUDIO_REC_START_ADDR;
    audio_buffer_in_B = (uint16_t*)(AUDIO_REC_START_ADDR + (audio_block_size));
    audio_buffer_out = (uint16_t*)(AUDIO_REC_START_ADDR + (audio_block_size * 2));
    audio_buffer_out_B = (uint16_t*)(AUDIO_REC_START_ADDR + (audio_block_size * 2) + (audio_block_size));

    memset(audio_buffer_in, 0, audio_block_size*2);
    memset(audio_buffer_out, 0, audio_block_size*2);
    audio_rec_buffer_state = BUFFER_OFFSET_NONE;

    /* Start Recording */
    BSP_AUDIO_IN_Record(audio_buffer_in, audio_block_size);

    /* Start Playback */
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    BSP_AUDIO_OUT_Play(audio_buffer_out, audio_block_size * 2);

    enc_in_idx = 0;
    enc_out_idx = 0;

    bytesPerFrame = (codec2_bits_per_frame(c2) | 7) >> 3;

    for (sec = 0; sec < latency; sec++) {
        uint16_t* inBuf;
        while (audio_rec_buffer_state == BUFFER_OFFSET_NONE)
            svc_uart();

        inBuf = (audio_rec_buffer_state == BUFFER_OFFSET_HALF) ? audio_buffer_in : audio_buffer_in_B;
        printf("encodeStart %u\r\n", enc_in_idx);
        decimated_encode((short*)inBuf);

        audio_rec_buffer_state = BUFFER_OFFSET_NONE;
    }

    fullAt = HAL_GetTick();
    sec = HAL_GetTick();
    while (1)
    {
        /* Wait end of half block recording */
        while(audio_rec_buffer_state != BUFFER_OFFSET_HALF)
        {
            svc_uart();
        }
        cpy_cnt++;
        halfAt = HAL_GetTick();
        if (halfAt - sec > 1000) {
            sec = halfAt;
            cpy_cnt = 0;
        }
        audio_rec_buffer_state = BUFFER_OFFSET_NONE;
        /* Copy recorded 1st half block */
        c2_passthru((short*)audio_buffer_out, (short*)audio_buffer_in);
        printf("navgs:%u nsamp:%u\r\n", navgs_, nsamp);

        /* Wait end of one block recording */
        while(audio_rec_buffer_state != BUFFER_OFFSET_FULL)
        {
            svc_uart();
        }
        cpy_cnt++;
        fullAt = HAL_GetTick();
        if (fullAt - sec > 1000) {
            sec = fullAt;
            cpy_cnt = 0;
        }
        audio_rec_buffer_state = BUFFER_OFFSET_NONE;
        /* Copy recorded 2nd half block */
        c2_passthru((short*)audio_buffer_out_B, (short*)audio_buffer_in_B);
        printf("\r\n");

        svc_uart();
    } // .. while(1)

} // ..AudioLoopback_demo()

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
