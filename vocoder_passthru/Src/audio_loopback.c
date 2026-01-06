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

uint8_t encoded[N_ENCS][OPUS_WRAPPER_MAX_FRAME_BYTES];
int encoded_len[N_ENCS];  /* actual encoded size for each frame */
int enc_in_idx, enc_out_idx;
uint8_t latency = 0;   // no initial buffer delay needed

bool bypass_codec = false;  /* 'b' to toggle: skip encode/decode, direct copy */
unsigned frame_count = 0;   /* count frames for diagnostics */
uint32_t enc_time_max = 0, dec_time_max = 0;  /* timing in ms */
volatile uint8_t dma_during_encode = 0;  /* set by DMA callback */
unsigned dma_overrun_count = 0;  /* count of DMA callbacks during encode */

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
            if (y >= (y_base+52) && y <= (y_base+78)) {
                pressed = PRESSED_DOWN;
            } else if (y >= y_base && y <= (y_base+26)) {
                pressed = PRESSED_UP;
            }
        }
    } else {
        if (pressed == PRESSED_UP) {
            if (++enc_in_idx == N_ENCS)
                enc_in_idx = 0;
        } else if (pressed == PRESSED_DOWN) {
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
    } else if (rxchar == 'b') {
        bypass_codec = !bypass_codec;
        printf("bypass_codec: %s\r\n", bypass_codec ? "ON (direct copy)" : "OFF (opus)");
    } else if (rxchar == '?') {
        int buf_fill = enc_in_idx - enc_out_idx;
        if (buf_fill < 0) buf_fill += N_ENCS;
        printf("frames:%u in:%d out:%d fill:%d bypass:%d navgs:%u nsamp:%u\r\n",
               frame_count, enc_in_idx, enc_out_idx, buf_fill, bypass_codec, navgs_, nsamp);
        /* Show last 4 encoded lengths */
        {
            int idx = enc_in_idx - 1;
            if (idx < 0) idx += N_ENCS;
            printf("enc_lens: %d %d %d %d\r\n",
                   encoded_len[idx],
                   encoded_len[(idx-1+N_ENCS)%N_ENCS],
                   encoded_len[(idx-2+N_ENCS)%N_ENCS],
                   encoded_len[(idx-3+N_ENCS)%N_ENCS]);
        }
        /* Show audio buffer samples: in[0-3], out[0-3] */
        printf("in: %d %d %d %d  out: %d %d %d %d\r\n",
               audio_buffer_in[0], audio_buffer_in[1], audio_buffer_in[2], audio_buffer_in[3],
               audio_buffer_out[0], audio_buffer_out[1], audio_buffer_out[2], audio_buffer_out[3]);
        /* Show timing: max encode/decode times in ms (40ms frame budget) */
        printf("enc_max:%lu dec_max:%lu ms (budget 40ms) overruns:%u\r\n", enc_time_max, dec_time_max, dma_overrun_count);
    } else if (rxchar == 't') {
        enc_time_max = 0;
        dec_time_max = 0;
        dma_overrun_count = 0;
        printf("timing reset\r\n");
    } else if (rxchar == 'R') {
        printf("Resetting...\r\n");
        HAL_Delay(10);  /* allow printf to complete */
        NVIC_SystemReset();
    }
}

void decimated_encode(const short *in)
{
    short to_encoder[OPUS_WRAPPER_SAMPLES_PER_FRAME_48K];  /* max size for 48kHz */
    unsigned i, x;
    int sum;
    int num_avgs = navgs_;
    int len;

    for (x = 0; x < nsamp; x++) {
        int avg;
        sum = 0;
        for (i = 0; i < navgs_; i++) {
            if (micRightEn && micLeftEn) {
                int v = *in++;
                v += *in++; // taking average of both microphones
                sum += (v / 2);
            } else if (micRightEn) {
                sum += *in++;
                in++;
            } else if (micLeftEn) {
                in++;
                sum += *in++;
            }
        }
        avg = sum / num_avgs;
        to_encoder[x] = avg;
    }

    {
        uint32_t t0 = HAL_GetTick();
        dma_during_encode = 0;  /* clear before encode */
        len = opus_wrapper_encode(ow, to_encoder, encoded[enc_in_idx]);
        if (dma_during_encode) {
            dma_overrun_count++;
        }
        uint32_t dt = HAL_GetTick() - t0;
        if (dt > enc_time_max) enc_time_max = dt;
    }
    if (len < 0) {
        printf("encode err %d\r\n", len);
    }
    encoded_len[enc_in_idx] = len;

    if (++enc_in_idx == N_ENCS)
        enc_in_idx = 0;
}


void opus_passthru(short *out, const short *in)
{
    static short decoded[OPUS_WRAPPER_SAMPLES_PER_FRAME_48K];  /* max size for 48kHz */
    unsigned i, x;

    frame_count++;

    if (bypass_codec) {
        /* Direct copy: verify audio path without codec */
        unsigned total_samples = nsamp * navgs_ * 2;  /* stereo */
        memcpy(out, in, total_samples * sizeof(short));
        return;
    }

    /* Decimate audio to Opus sample rate */
    decimated_encode(in);

    /* Decode */
    {
        uint32_t t0 = HAL_GetTick();
        int dec_ret = opus_wrapper_decode(ow, encoded[enc_out_idx], encoded_len[enc_out_idx], decoded);
        uint32_t dt = HAL_GetTick() - t0;
        if (dt > dec_time_max) dec_time_max = dt;
        if (dec_ret < 0) {
            printf("decode err %d (len=%d)\r\n", dec_ret, encoded_len[enc_out_idx]);
        }
    }
    if (++enc_out_idx == N_ENCS)
        enc_out_idx = 0;

    /* Output decoded samples (with optional interpolation when navgs_ > 1) */
    for (x = 0; x < nsamp; x++) {
        short y0 = decoded[x];
        short dy = 0;

        if (navgs_ > 1 && x < (nsamp - 1)) {
            dy = (decoded[x+1] - y0) * step;
        }

        for (i = 0; i < navgs_; i++) {
            *out++ = y0;
            *out++ = y0;
            y0 += dy;
        }
    }
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

    {
        int opus_sr = opus_wrapper_get_sample_rate(ow);
        /* navgs_ = decimation ratio from audio rate to Opus sample rate */
        navgs_ = audioRate / opus_sr;
        if (navgs_ < 1) navgs_ = 1;  /* no upsampling supported */
        audio_block_size = nsamp * navgs_ * 4;  /* stereo, 2 bytes per sample */
        step = 1.0 / navgs_;
        printf("audioRate=%u, opus_sr=%d, navgs=%u, nsamp=%u, block=%u, complexity=%d\r\n",
               audioRate, opus_sr, navgs_, nsamp, audio_block_size, opus_wrapper_get_complexity(ow));
    }

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

    for (sec = 0; sec < latency; sec++) {
        uint16_t* inBuf;
        while (audio_rec_buffer_state == BUFFER_OFFSET_NONE)
            svc_uart();

        inBuf = (audio_rec_buffer_state == BUFFER_OFFSET_HALF) ? audio_buffer_in : audio_buffer_in_B;
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
        opus_passthru((short*)audio_buffer_out, (short*)audio_buffer_in);

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
        opus_passthru((short*)audio_buffer_out_B, (short*)audio_buffer_in_B);

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
