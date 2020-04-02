/**
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "string.h"
#include "radio.h"

#define DEFAULT_MIC_GAIN        95

#define SINE_TABLE_LENGTH       1024
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
volatile uint8_t idx_start; // tmp dbg

uint16_t *audio_buffer_in;
uint16_t *audio_buffer_in_B;
uint16_t *audio_buffer_out;
uint16_t *audio_buffer_out_B;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t  audio_rec_buffer_state;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t audio_in_vol = DEFAULT_AUDIO_IN_VOLUME;
uint8_t audio_out_vol = 64;

uint8_t currently_decoding;

typedef enum {
    PRESSED_NONE = 0,
    PRESSED_BW_UP,
    PRESSED_BW_DOWN,
    PRESSED_SF_UP,
    PRESSED_SF_DOWN,
    PRESSED_MICGAIN_UP,
    PRESSED_MICGAIN_DOWN,
    PRESSED_PKTLEN_UP,
    PRESSED_PKTLEN_DOWN
} pressed_e;

typedef enum {
    AUDIO_PLAY_NONE = 0,
    AUDIO_PLAY_HALF,
    AUDIO_PLAY_FULL
} AudioPlay_e;

AudioPlay_e audio_play_state_;

volatile uint32_t terminate_spkr_at_tick;

volatile uint8_t user_button_pressed;   // flag
volatile uint8_t txing;   // flag
volatile uint8_t rx_start_at_tx_done;   // flag
volatile uint8_t terminate_spkr_rx;   // flag

volatile uint32_t cycleDur;
volatile uint32_t cycleStartAt;

uint8_t frameCnt;
unsigned tickAtFrameSecond;

const uint16_t sine_table[SINE_TABLE_LENGTH] =
{
    0x8000,0x80c9,0x8192,0x825b,0x8324,0x83ee,0x84b7,0x8580,
    0x8649,0x8712,0x87db,0x88a4,0x896c,0x8a35,0x8afe,0x8bc6,
    0x8c8e,0x8d57,0x8e1f,0x8ee7,0x8fae,0x9076,0x913e,0x9205,
    0x92cc,0x9393,0x945a,0x9521,0x95e7,0x96ad,0x9773,0x9839,
    0x98fe,0x99c4,0x9a89,0x9b4d,0x9c12,0x9cd6,0x9d9a,0x9e5e,
    0x9f21,0x9fe4,0xa0a7,0xa169,0xa22b,0xa2ed,0xa3af,0xa470,
    0xa530,0xa5f1,0xa6b1,0xa770,0xa830,0xa8ef,0xa9ad,0xaa6b,
    0xab29,0xabe6,0xaca3,0xad5f,0xae1b,0xaed7,0xaf92,0xb04d,
    0xb107,0xb1c0,0xb27a,0xb332,0xb3ea,0xb4a2,0xb559,0xb610,
    0xb6c6,0xb77c,0xb831,0xb8e5,0xb999,0xba4d,0xbb00,0xbbb2,
    0xbc64,0xbd15,0xbdc6,0xbe76,0xbf25,0xbfd4,0xc082,0xc12f,
    0xc1dc,0xc288,0xc334,0xc3df,0xc489,0xc533,0xc5dc,0xc684,
    0xc72c,0xc7d3,0xc879,0xc91f,0xc9c3,0xca67,0xcb0b,0xcbae,
    0xcc4f,0xccf1,0xcd91,0xce31,0xced0,0xcf6e,0xd00b,0xd0a8,
    0xd144,0xd1df,0xd279,0xd313,0xd3ac,0xd443,0xd4db,0xd571,
    0xd606,0xd69b,0xd72f,0xd7c2,0xd854,0xd8e5,0xd975,0xda05,
    0xda93,0xdb21,0xdbae,0xdc3a,0xdcc5,0xdd4f,0xddd9,0xde61,
    0xdee9,0xdf6f,0xdff5,0xe07a,0xe0fd,0xe180,0xe202,0xe283,
    0xe303,0xe382,0xe400,0xe47d,0xe4fa,0xe575,0xe5ef,0xe668,
    0xe6e0,0xe758,0xe7ce,0xe843,0xe8b7,0xe92b,0xe99d,0xea0e,
    0xea7e,0xeaed,0xeb5b,0xebc8,0xec34,0xec9f,0xed09,0xed72,
    0xedda,0xee41,0xeea7,0xef0b,0xef6f,0xefd1,0xf033,0xf093,
    0xf0f2,0xf150,0xf1ad,0xf209,0xf264,0xf2be,0xf316,0xf36e,
    0xf3c4,0xf41a,0xf46e,0xf4c1,0xf513,0xf564,0xf5b3,0xf602,
    0xf64f,0xf69b,0xf6e6,0xf730,0xf779,0xf7c1,0xf807,0xf84d,
    0xf891,0xf8d4,0xf916,0xf956,0xf996,0xf9d4,0xfa11,0xfa4d,
    0xfa88,0xfac1,0xfafa,0xfb31,0xfb67,0xfb9c,0xfbd0,0xfc02,
    0xfc33,0xfc63,0xfc92,0xfcc0,0xfcec,0xfd17,0xfd42,0xfd6a,
    0xfd92,0xfdb8,0xfdde,0xfe01,0xfe24,0xfe46,0xfe66,0xfe85,
    0xfea3,0xfec0,0xfedb,0xfef5,0xff0e,0xff26,0xff3c,0xff52,
    0xff66,0xff79,0xff8a,0xff9b,0xffaa,0xffb8,0xffc4,0xffd0,
    0xffda,0xffe3,0xffeb,0xfff1,0xfff6,0xfffa,0xfffd,0xffff,
    0xffff,0xfffe,0xfffc,0xfff8,0xfff4,0xffee,0xffe7,0xffdf,
    0xffd5,0xffca,0xffbe,0xffb1,0xffa2,0xff93,0xff82,0xff6f,
    0xff5c,0xff47,0xff31,0xff1a,0xff02,0xfee8,0xfece,0xfeb1,
    0xfe94,0xfe76,0xfe56,0xfe35,0xfe13,0xfdf0,0xfdcb,0xfda5,
    0xfd7e,0xfd56,0xfd2d,0xfd02,0xfcd6,0xfca9,0xfc7b,0xfc4b,
    0xfc1b,0xfbe9,0xfbb6,0xfb82,0xfb4c,0xfb16,0xfade,0xfaa5,
    0xfa6b,0xfa2f,0xf9f3,0xf9b5,0xf976,0xf936,0xf8f5,0xf8b2,
    0xf86f,0xf82a,0xf7e4,0xf79d,0xf755,0xf70c,0xf6c1,0xf675,
    0xf629,0xf5db,0xf58c,0xf53b,0xf4ea,0xf498,0xf444,0xf3ef,
    0xf399,0xf342,0xf2ea,0xf291,0xf237,0xf1db,0xf17f,0xf121,
    0xf0c3,0xf063,0xf002,0xefa0,0xef3d,0xeed9,0xee74,0xee0e,
    0xeda6,0xed3e,0xecd5,0xec6a,0xebff,0xeb92,0xeb24,0xeab6,
    0xea46,0xe9d6,0xe964,0xe8f1,0xe87d,0xe809,0xe793,0xe71c,
    0xe6a4,0xe62c,0xe5b2,0xe537,0xe4bc,0xe43f,0xe3c1,0xe343,
    0xe2c3,0xe243,0xe1c1,0xe13f,0xe0bc,0xe037,0xdfb2,0xdf2c,
    0xdea5,0xde1d,0xdd94,0xdd0a,0xdc80,0xdbf4,0xdb68,0xdada,
    0xda4c,0xd9bd,0xd92d,0xd89c,0xd80b,0xd778,0xd6e5,0xd651,
    0xd5bc,0xd526,0xd48f,0xd3f8,0xd35f,0xd2c6,0xd22c,0xd192,
    0xd0f6,0xd05a,0xcfbd,0xcf1f,0xce80,0xcde1,0xcd41,0xcca0,
    0xcbff,0xcb5c,0xcab9,0xca16,0xc971,0xc8cc,0xc826,0xc77f,
    0xc6d8,0xc630,0xc588,0xc4de,0xc434,0xc38a,0xc2de,0xc232,
    0xc186,0xc0d9,0xc02b,0xbf7c,0xbecd,0xbe1e,0xbd6d,0xbcbd,
    0xbc0b,0xbb59,0xbaa6,0xb9f3,0xb940,0xb88b,0xb7d6,0xb721,
    0xb66b,0xb5b5,0xb4fe,0xb446,0xb38e,0xb2d6,0xb21d,0xb164,
    0xb0aa,0xafef,0xaf34,0xae79,0xadbd,0xad01,0xac45,0xab88,
    0xaaca,0xaa0c,0xa94e,0xa88f,0xa7d0,0xa711,0xa651,0xa591,
    0xa4d0,0xa40f,0xa34e,0xa28c,0xa1ca,0xa108,0xa045,0x9f83,
    0x9ebf,0x9dfc,0x9d38,0x9c74,0x9bb0,0x9aeb,0x9a26,0x9961,
    0x989c,0x97d6,0x9710,0x964a,0x9584,0x94bd,0x93f7,0x9330,
    0x9269,0x91a1,0x90da,0x9012,0x8f4b,0x8e83,0x8dbb,0x8cf3,
    0x8c2a,0x8b62,0x8a99,0x89d1,0x8908,0x883f,0x8776,0x86ad,
    0x85e4,0x851b,0x8452,0x8389,0x82c0,0x81f7,0x812d,0x8064,
    0x7f9b,0x7ed2,0x7e08,0x7d3f,0x7c76,0x7bad,0x7ae4,0x7a1b,
    0x7952,0x7889,0x77c0,0x76f7,0x762e,0x7566,0x749d,0x73d5,
    0x730c,0x7244,0x717c,0x70b4,0x6fed,0x6f25,0x6e5e,0x6d96,
    0x6ccf,0x6c08,0x6b42,0x6a7b,0x69b5,0x68ef,0x6829,0x6763,
    0x669e,0x65d9,0x6514,0x644f,0x638b,0x62c7,0x6203,0x6140,
    0x607c,0x5fba,0x5ef7,0x5e35,0x5d73,0x5cb1,0x5bf0,0x5b2f,
    0x5a6e,0x59ae,0x58ee,0x582f,0x5770,0x56b1,0x55f3,0x5535,
    0x5477,0x53ba,0x52fe,0x5242,0x5186,0x50cb,0x5010,0x4f55,
    0x4e9b,0x4de2,0x4d29,0x4c71,0x4bb9,0x4b01,0x4a4a,0x4994,
    0x48de,0x4829,0x4774,0x46bf,0x460c,0x4559,0x44a6,0x43f4,
    0x4342,0x4292,0x41e1,0x4132,0x4083,0x3fd4,0x3f26,0x3e79,
    0x3dcd,0x3d21,0x3c75,0x3bcb,0x3b21,0x3a77,0x39cf,0x3927,
    0x3880,0x37d9,0x3733,0x368e,0x35e9,0x3546,0x34a3,0x3400,
    0x335f,0x32be,0x321e,0x317f,0x30e0,0x3042,0x2fa5,0x2f09,
    0x2e6d,0x2dd3,0x2d39,0x2ca0,0x2c07,0x2b70,0x2ad9,0x2a43,
    0x29ae,0x291a,0x2887,0x27f4,0x2763,0x26d2,0x2642,0x25b3,
    0x2525,0x2497,0x240b,0x237f,0x22f5,0x226b,0x21e2,0x215a,
    0x20d3,0x204d,0x1fc8,0x1f43,0x1ec0,0x1e3e,0x1dbc,0x1d3c,
    0x1cbc,0x1c3e,0x1bc0,0x1b43,0x1ac8,0x1a4d,0x19d3,0x195b,
    0x18e3,0x186c,0x17f6,0x1782,0x170e,0x169b,0x1629,0x15b9,
    0x1549,0x14db,0x146d,0x1400,0x1395,0x132a,0x12c1,0x1259,
    0x11f1,0x118b,0x1126,0x10c2,0x105f,0xffd,0xf9c,0xf3c,
    0xede,0xe80,0xe24,0xdc8,0xd6e,0xd15,0xcbd,0xc66,
    0xc10,0xbbb,0xb67,0xb15,0xac4,0xa73,0xa24,0x9d6,
    0x98a,0x93e,0x8f3,0x8aa,0x862,0x81b,0x7d5,0x790,
    0x74d,0x70a,0x6c9,0x689,0x64a,0x60c,0x5d0,0x594,
    0x55a,0x521,0x4e9,0x4b3,0x47d,0x449,0x416,0x3e4,
    0x3b4,0x384,0x356,0x329,0x2fd,0x2d2,0x2a9,0x281,
    0x25a,0x234,0x20f,0x1ec,0x1ca,0x1a9,0x189,0x16b,
    0x14e,0x131,0x117,0xfd,0xe5,0xce,0xb8,0xa3,
    0x90,0x7d,0x6c,0x5d,0x4e,0x41,0x35,0x2a,
    0x20,0x18,0x11,0xb,0x7,0x3,0x1,0x0,
    0x0,0x2,0x5,0x9,0xe,0x14,0x1c,0x25,
    0x2f,0x3b,0x47,0x55,0x64,0x75,0x86,0x99,
    0xad,0xc3,0xd9,0xf1,0x10a,0x124,0x13f,0x15c,
    0x17a,0x199,0x1b9,0x1db,0x1fe,0x221,0x247,0x26d,
    0x295,0x2bd,0x2e8,0x313,0x33f,0x36d,0x39c,0x3cc,
    0x3fd,0x42f,0x463,0x498,0x4ce,0x505,0x53e,0x577,
    0x5b2,0x5ee,0x62b,0x669,0x6a9,0x6e9,0x72b,0x76e,
    0x7b2,0x7f8,0x83e,0x886,0x8cf,0x919,0x964,0x9b0,
    0x9fd,0xa4c,0xa9b,0xaec,0xb3e,0xb91,0xbe5,0xc3b,
    0xc91,0xce9,0xd41,0xd9b,0xdf6,0xe52,0xeaf,0xf0d,
    0xf6c,0xfcc,0x102e,0x1090,0x10f4,0x1158,0x11be,0x1225,
    0x128d,0x12f6,0x1360,0x13cb,0x1437,0x14a4,0x1512,0x1581,
    0x15f1,0x1662,0x16d4,0x1748,0x17bc,0x1831,0x18a7,0x191f,
    0x1997,0x1a10,0x1a8a,0x1b05,0x1b82,0x1bff,0x1c7d,0x1cfc,
    0x1d7c,0x1dfd,0x1e7f,0x1f02,0x1f85,0x200a,0x2090,0x2116,
    0x219e,0x2226,0x22b0,0x233a,0x23c5,0x2451,0x24de,0x256c,
    0x25fa,0x268a,0x271a,0x27ab,0x283d,0x28d0,0x2964,0x29f9,
    0x2a8e,0x2b24,0x2bbc,0x2c53,0x2cec,0x2d86,0x2e20,0x2ebb,
    0x2f57,0x2ff4,0x3091,0x312f,0x31ce,0x326e,0x330e,0x33b0,
    0x3451,0x34f4,0x3598,0x363c,0x36e0,0x3786,0x382c,0x38d3,
    0x397b,0x3a23,0x3acc,0x3b76,0x3c20,0x3ccb,0x3d77,0x3e23,
    0x3ed0,0x3f7d,0x402b,0x40da,0x4189,0x4239,0x42ea,0x439b,
    0x444d,0x44ff,0x45b2,0x4666,0x471a,0x47ce,0x4883,0x4939,
    0x49ef,0x4aa6,0x4b5d,0x4c15,0x4ccd,0x4d85,0x4e3f,0x4ef8,
    0x4fb2,0x506d,0x5128,0x51e4,0x52a0,0x535c,0x5419,0x54d6,
    0x5594,0x5652,0x5710,0x57cf,0x588f,0x594e,0x5a0e,0x5acf,
    0x5b8f,0x5c50,0x5d12,0x5dd4,0x5e96,0x5f58,0x601b,0x60de,
    0x61a1,0x6265,0x6329,0x63ed,0x64b2,0x6576,0x663b,0x6701,
    0x67c6,0x688c,0x6952,0x6a18,0x6ade,0x6ba5,0x6c6c,0x6d33,
    0x6dfa,0x6ec1,0x6f89,0x7051,0x7118,0x71e0,0x72a8,0x7371,
    0x7439,0x7501,0x75ca,0x7693,0x775b,0x7824,0x78ed,0x79b6,
    0x7a7f,0x7b48,0x7c11,0x7cdb,0x7da4,0x7e6d,0x7f36,0x8000
};

/**
  * @brief  Audio IN Error callback function.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_Error_CallBack(void)
{
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
  /* Display message on the LCD screen */
  BSP_LCD_SetBackColor(LCD_COLOR_RED);
  BSP_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)"       DMA  ERROR     ", CENTER_MODE);

  /* Stop the program with an infinite loop */
  while (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    return;
  }
  /* could also generate a system reset to recover from the error */
  /* .... */
}

/**
  * @brief Manages the DMA Transfer complete interrupt.
  * @param None
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
    audio_rec_buffer_state = BUFFER_OFFSET_FULL;
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
    audio_rec_buffer_state = BUFFER_OFFSET_HALF;
}

#define LFSR_INIT       0x1ff
unsigned noise_lfsr = LFSR_INIT;

uint8_t get_pn9_byte(unsigned* lfsr)
{
    uint8_t ret = 0;
    int xor_out;

    xor_out = ((*lfsr >> 5) & 0xf) ^ (*lfsr & 0xf);   // four bits at a time
    *lfsr = (*lfsr >> 4) | (xor_out << 5);    // four bits at a time

    ret |= (*lfsr >> 5) & 0x0f;

    xor_out = ((*lfsr >> 5) & 0xf) ^ (*lfsr & 0xf);   // four bits at a time
    *lfsr = (*lfsr >> 4) | (xor_out << 5);    // four bits at a time

    ret |= ((*lfsr >> 1) & 0xf0);

    return ret;
}

void BSP_AUDIO_OUT_TransferComplete_CallBack()
{
    audio_play_state_ = AUDIO_PLAY_FULL;
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack()
{
    audio_play_state_ = AUDIO_PLAY_HALF;
}

void svc_uart()
{
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
    } else if (rxchar == '.') {
        appHal.radioPrintStatus();
    }
}

#define N_HISTORY       12
void decimated_encode(const short *in, uint8_t *out)
{
    short to_encoder[320];
    unsigned i, x;
    int sum;

    for (x = 0; x < nsamp; x++) {
        sum = 0;
        for (i = 0; i < navgs_; i++) {
            if (micRightEn && micLeftEn) {
                int v = *in++;
                v += *in++;
                sum += (v / 2);
            } else if (micRightEn) {
                sum += *in++;
                in++;
            } else if (micLeftEn) {
                in++;
                sum += *in++;
            }
        }
        to_encoder[x] = sum / navgs_;
    }

    codec2_encode(c2, out, to_encoder);
    if (++frameCnt == frames_per_sec) {
        printf("%lums for %u frames\r\n", uwTick - tickAtFrameSecond, frames_per_sec);
        tickAtFrameSecond = uwTick;
        frameCnt = 0;
    }
}

void tx_encoded(uint8_t tx_nbytes)
{
    lorahal.service();

    if (txing) {
        printf("\e[31mstillTxing\e[0m\r\n");
        lcd_print_tx_duration(0, (unsigned)cycleDur);
    }

    txing = 1;
    txStartAt = uwTick;
    lorahal.send(tx_nbytes);
    appHal.lcd_printOpMode(true);
}

void lora_rx_begin()
{
    lorahal.rx(0);
    appHal.lcd_printOpMode(false);
    frameCnt = 0;
}

void sine_out(unsigned skipcnt)
{
    short *outPtr = NULL;
    unsigned table_idx;

    skipcnt /= 2;

    if (audio_play_state_ == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (audio_play_state_ == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    audio_play_state_ = AUDIO_PLAY_NONE;

    table_idx = 0;
    for (unsigned i = 0; i < audio_block_size; i++) {
        int16_t out = (int16_t)(sine_table[table_idx] - 0x8000);
        out /= 4;

        *outPtr++ = out;
        *outPtr++ = out;

        table_idx += skipcnt;
        if (table_idx > SINE_TABLE_LENGTH)
            table_idx -= SINE_TABLE_LENGTH;
    }
} // ..sine_out()

void start_tone()
{
    sine_out((rx_rssi+(rx_snr*4)) + 170);
}

void silence()
{
    short *outPtr = NULL;

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    if (audio_play_state_ == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (audio_play_state_ == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    audio_play_state_ = AUDIO_PLAY_NONE;

    memset(outPtr, 0, audio_block_size);

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    if (audio_play_state_ == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (audio_play_state_ == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    audio_play_state_ = AUDIO_PLAY_NONE;

    memset(outPtr, 0, audio_block_size);
} // ..silence()

void tone_out(uint8_t mask)
{
    short *outPtr = NULL;

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    if (audio_play_state_ == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (audio_play_state_ == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    audio_play_state_ = AUDIO_PLAY_NONE;

    for (unsigned i = 0; i < audio_block_size; i++) {
        short out;
        if (mask == 0xff) {
            out = get_pn9_byte(&noise_lfsr);
            out ^= get_pn9_byte(&noise_lfsr);
            out <<= 8;
            out += get_pn9_byte(&noise_lfsr);
            out &= 0x1fff; // reduce volume
            *outPtr++ = out;
            *outPtr++ = out;
        }
    }
}

void end_rx_tone()
{
    currently_decoding = 0;
    tone_out(0xff);   // 0xff noise
    silence();
    lcd_print_rssi_snr(0, 0);
}

short *decBuf, *prevDecBuf;
uint8_t fdb;

void put_spkr()
{
    unsigned x;
    short *outPtr = NULL;

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    if (audio_play_state_ == AUDIO_PLAY_FULL) {
        //printf("playFull\r\n");
        outPtr = (short*)audio_buffer_out_B;
    } else if (audio_play_state_ == AUDIO_PLAY_HALF) {
        //printf("playHalf\r\n");
        outPtr = (short*)audio_buffer_out;
    }

    audio_play_state_ = AUDIO_PLAY_NONE;

    for (x = 0; x < nsamp; x++) {
        short y0 = prevDecBuf[x];
        short dy;
        if (x < (nsamp-1))
            dy = (prevDecBuf[x+1] - y0) * step;
        else
            dy = (decBuf[0] - y0) * step;

        for (unsigned i = 0; i < navgs_; i++) {
            short v = y0 += dy;
            *outPtr++ = v;
            *outPtr++ = v;
        }
    }

    fdb ^= 1;

    if (++frameCnt == frames_per_sec) {
        printf("%lums for %u frames\r\n", uwTick - tickAtFrameSecond, frames_per_sec);
        tickAtFrameSecond = uwTick;
        frameCnt = 0;
    }
}

void parse_rx()
{
    unsigned n;
    static short from_decoderA[320];
    static short from_decoderB[320];

    if (currently_decoding == 0) {
        fdb = 0;
        start_tone();
        currently_decoding = 1;
    } else {
        /* successive received packet */
        terminate_spkr_at_tick = 0;
    }

    if (selected_bitrate == CODEC2_MODE_700C || selected_bitrate == CODEC2_MODE_1300) {
        for (n = 0; n < rx_size; n += _bytes_per_frame) {
            unsigned i;
            uint8_t scratch[7];
            printf("%u,%u) ", n, frame_length_bytes);
            for (i = 0; i < _bytes_per_frame; i++)
                printf("%02x ", lorahal.rx_buf[n+i]);
            printf("\r\n");
            for (i = 0; i < frame_length_bytes; i++)
                scratch[i] = lorahal.rx_buf[n+i];
            scratch[i] = lorahal.rx_buf[n+i] & 0xf0;
            printf("A ");
            for (i = 0; i <= frame_length_bytes; i++)
                printf("%02x ", scratch[i]);

            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, scratch);
            put_spkr();

            for (i = 0; i < frame_length_bytes; i++) {
                scratch[i] = lorahal.rx_buf[n+i+frame_length_bytes] & 0x0f;
                scratch[i] <<= 4;
                if (i < frame_length_bytes)
                    scratch[i] |= (lorahal.rx_buf[n+i+frame_length_bytes+1] & 0xf0) >> 4;
            }
            printf("B ");
            for (i = 0; i <= frame_length_bytes; i++)
                printf("%02x ", scratch[i]);

            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, scratch);
            put_spkr();
        }

    } else if (selected_bitrate == CODEC2_MODE_450 || selected_bitrate == CODEC2_MODE_450PWB) {
        for (n = 0; n < rx_size; /*n += _bytes_per_frame*/) {
            unsigned i, stop = n + _bytes_per_frame;
            if (stop > rx_size)
                stop = rx_size;
            for (i = n; i < stop; i++)
                printf("%02x ", lorahal.rx_buf[i]);
            printf("\r\n");
            uint8_t o, scratch[3];  // {} = 18bits to decoder
            scratch[0] = lorahal.rx_buf[n++];         // n=0 { 0, 1, 2, 3, 4, 5, 6, 7}
            scratch[1] = lorahal.rx_buf[n++];         // n=1 { 8, 9,10,11,12,13,14,15}
            scratch[2] = lorahal.rx_buf[n] & 0xc0;    // n=2 {16,17}
            if (n >= rx_size) {
                printf("cutoffA\r\n");
                break;
            }
            printf("A) %02x %02x %02x\r\n", scratch[0], scratch[1], scratch[2]);
            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, scratch);
            put_spkr();
            scratch[0] = lorahal.rx_buf[n++] & 0x3f;  // n=2 {0,1,2,3,4,5}
            scratch[0] <<= 2;
            o = lorahal.rx_buf[n] & 0xc0;             // n=3 {6,7}
            o >>= 6;
            scratch[0] |= o;
            scratch[1] = lorahal.rx_buf[n++] & 0x3f;  // n=3 {8,9,10,11,12,13}
            scratch[1] <<= 2;
            o = lorahal.rx_buf[n] & 0xc0;             // n=4 {14,15}
            o >>= 6;
            scratch[1] |= o;
            o = lorahal.rx_buf[n] & 0x30;             // n=4 {16,17}
            o <<= 2;
            scratch[2] = o;
            if (n >= rx_size) {
                printf("cutoffB\r\n");
                break;
            }
            printf("B) %02x %02x %02x\r\n", scratch[0], scratch[1], scratch[2]);
            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, scratch);
            put_spkr();
            o = lorahal.rx_buf[n++] & 0x0f;             // n=4 {0,1,2,3}
            o <<= 4;
            scratch[0] = o;
            o = lorahal.rx_buf[n] & 0xf0;             // n=5 {4,5,6,7}
            o >>= 4;
            scratch[0] |= o;
            o = lorahal.rx_buf[n++] & 0x0f;             // n=5 {8,9,10,11}
            o <<= 4;
            scratch[1] = o;
            o = lorahal.rx_buf[n] & 0xf0;             // n=6 {12,13,14,15}
            o >>= 4;
            scratch[1] |= o;
            o = lorahal.rx_buf[n] & 0x0c;            // n=6 {16,17}
            o <<= 4;
            scratch[2] = o;
            if (n >= rx_size) {
                printf("cutoffC\r\n");
                break;
            }
            printf("C) %02x %02x %02x\r\n", scratch[0], scratch[1], scratch[2]);
            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, scratch);
            put_spkr();
            scratch[0] = lorahal.rx_buf[n++] & 0x03;            // n=6 {0,1}
            scratch[0] <<= 6;
            o = lorahal.rx_buf[n] & 0xfc;   // n=7 {2,3,4,5,6,7}
            o >>= 2;
            scratch[0] |= o;
            o = lorahal.rx_buf[n++] & 0x03;    // n=7 {8,9}
            o <<= 6;
            scratch[1] = o;
            o = lorahal.rx_buf[n] & 0xfc;    // n=8 {10,11,12,13,14,15}
            o >>= 2;
            scratch[1] |= o;
            scratch[2] = lorahal.rx_buf[n++] & 0x03;  // n=8 {16,17}
            scratch[2] <<= 6;
            printf("D) %02x %02x %02x\r\nrx_in:", scratch[0], scratch[1], scratch[2]);
            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, scratch);
            put_spkr();
        }
    } else {
        for (n = 0; n < rx_size; n += _bytes_per_frame) {
            const uint8_t *encoded = &lorahal.rx_buf[n];
            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;
            codec2_decode(c2, decBuf, encoded);

            //printf("%u,%uplayStateWait\r\n", fdb, n);
            put_spkr();
        }
    }

    if (rx_size < lora_payload_length) {
        end_rx_tone();
    } else {
        terminate_spkr_at_tick = uwTick + inter_pkt_timeout;
    }
} // ..parse_rx()

void HAL_IncTick(void)
{
    uwTick += uwTickFreq;

    if (uwTick == terminate_spkr_at_tick) {
        /* radio receiver end-of-transmission : for last pkt full length */
        terminate_spkr_rx = 1;
    }
} 

void delay_ticks(unsigned t)
{
    unsigned dest = uwTick + t;
    while (uwTick < dest)
        asm("nop");
}

static void mic_LCD_print(unsigned x_base, unsigned y_base)
{
    char str[16];
    sprintf(str, "mic:%u ", AudioInVolume);
    BSP_LCD_DisplayStringAt(x_base-35, y_base+52, (uint8_t *)str, LEFT_MODE);
}

void radio_screen(const TS_StateTypeDef *TS_State)
{
    pressed_e pressed = PRESSED_NONE;
    static pressed_e prevPressed;
    const unsigned y_base = 30;
    const unsigned x_sf_base = 180;
    const unsigned y_pktlen_base = y_base + 130;
    const unsigned x_micgain_base = BSP_LCD_GetXSize() - 80;

    BSP_LCD_SetFont(&Font24);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

    appHal.lcd_print_bw(0, y_base+52);
    appHal.lcd_print_sf(x_sf_base, y_base+52);
    mic_LCD_print(x_micgain_base, y_base);
    {
        char str[16];
        sprintf(str, "%u pktLength  ", lora_payload_length);
        BSP_LCD_DisplayStringAt(0, y_pktlen_base+26, (uint8_t *)str, LEFT_MODE);
    }

    lorahal.service();
    if (TS_State->touchDetected) {
        uint16_t x, y;
        x = TS_State->touchX[0];
        y = TS_State->touchY[0];
        if (x < x_sf_base-20) {
            if (y > (y_base+26) && y < (y_base+52)) {   // 56 to 72
                pressed = PRESSED_BW_UP;
            } else if (y > (y_base+78) && y < (y_base+104)) { // 108 to 134
                pressed = PRESSED_BW_DOWN;
            } else if (y >= y_pktlen_base && y < y_pktlen_base+26) {
                pressed = PRESSED_PKTLEN_UP;
            } else if (y >= y_pktlen_base+52 && y < y_pktlen_base+78) {
                pressed = PRESSED_PKTLEN_DOWN;
            }
        } else if (x < x_micgain_base-20) {
            if (y > (y_base+26) && y < (y_base+52)) {   // 56 to 72
                pressed = PRESSED_SF_UP;
            } else if (y > (y_base+78) && y < (y_base+104)) { // 108 to 134
                pressed = PRESSED_SF_DOWN;
            }
        } else {
            if (y > (y_base+26) && y < (y_base+52)) {   // 56 to 72
                pressed = PRESSED_MICGAIN_UP;
            } else if (y > (y_base+78) && y < (y_base+104)) { // 108 to 134
                pressed = PRESSED_MICGAIN_DOWN;
            }
        }
    } else {
        if (prevPressed == PRESSED_BW_DOWN || prevPressed == PRESSED_BW_UP) {
            lorahal.standby();
        }

        if (prevPressed == PRESSED_BW_DOWN && !lorahal.bw_at_lowest() && !lorahal.sf_at_fastest()) {
            appHal.step_bw(false);
            appHal.lcd_print_bw(0, y_base+52);
            appHal.step_sf(true); // keep same datarate, sf up
        } else if (prevPressed == PRESSED_BW_UP && !lorahal.bw_at_highest() && !lorahal.sf_at_slowest()) {
            appHal.step_bw(true);
            appHal.lcd_print_bw(0, y_base+52);
            appHal.step_sf(false);    // keep same datarate, sf down
        } else if (prevPressed == PRESSED_SF_DOWN) {
            appHal.step_sf(false);
            appHal.lcd_print_sf(x_sf_base, y_base+52);
        } else if (prevPressed == PRESSED_SF_UP) {
            appHal.step_sf(true);
            appHal.lcd_print_sf(x_sf_base, y_base+52);
        } else if (prevPressed == PRESSED_MICGAIN_UP) {
            if (AudioInVolume < 100)
                BSP_AUDIO_IN_SetVolume(AudioInVolume+1);
            mic_LCD_print(x_micgain_base, y_base);
        } else if (prevPressed == PRESSED_MICGAIN_DOWN) {
            if (AudioInVolume > 0)
                BSP_AUDIO_IN_SetVolume(AudioInVolume-1);
            mic_LCD_print(x_micgain_base, y_base);
        } else if (prevPressed == PRESSED_PKTLEN_UP) {
            if (lora_payload_length < 256-_bytes_per_frame)
                lora_payload_length += _bytes_per_frame;
            printf("pktLenUp %u\r\n", lora_payload_length );
        } else if (prevPressed == PRESSED_PKTLEN_DOWN) {
            if (lora_payload_length > _bytes_per_frame)
                lora_payload_length -= _bytes_per_frame;
            printf("pktLenDown %u\r\n", lora_payload_length);
        }

        if (prevPressed == PRESSED_BW_DOWN || prevPressed == PRESSED_BW_UP) {
            lora_rx_begin();
        }
    }

    lorahal.service();

    BSP_LCD_SetBackColor(pressed == PRESSED_BW_UP ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_BW_UP ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, y_base+26, (uint8_t *)" UP", LEFT_MODE);

    BSP_LCD_SetBackColor(pressed == PRESSED_BW_DOWN ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_BW_DOWN ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, y_base+78, (uint8_t *)"DOWN", LEFT_MODE);


    BSP_LCD_SetBackColor(pressed == PRESSED_SF_UP ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_SF_UP ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(x_sf_base, y_base+26, (uint8_t *)" UP", LEFT_MODE);

    BSP_LCD_SetBackColor(pressed == PRESSED_SF_DOWN ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_SF_DOWN ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(x_sf_base, y_base+78, (uint8_t *)"DOWN", LEFT_MODE);


    BSP_LCD_SetBackColor(pressed == PRESSED_MICGAIN_UP ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_MICGAIN_UP ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(x_micgain_base, y_base+26, (uint8_t *)" UP", LEFT_MODE);

    BSP_LCD_SetBackColor(pressed == PRESSED_MICGAIN_DOWN ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_MICGAIN_DOWN ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(x_micgain_base, y_base+78, (uint8_t *)"DOWN", LEFT_MODE);


    BSP_LCD_SetBackColor(pressed == PRESSED_PKTLEN_UP ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_PKTLEN_UP ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, y_pktlen_base, (uint8_t *)" UP", LEFT_MODE);

    BSP_LCD_SetBackColor(pressed == PRESSED_PKTLEN_DOWN ? LCD_COLOR_BLUE : LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(pressed == PRESSED_PKTLEN_DOWN ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, y_pktlen_base+52, (uint8_t *)"DOWN", LEFT_MODE);

    prevPressed = pressed;
}

/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
void AudioLoopback_demo(void)
{
    uint8_t scratch[8];
    uint8_t mid = 0;
    TS_StateTypeDef prev_TS_State;
    uint8_t tx_buf_idx = 0;

    BSP_LCD_SetFont(&Font12);
    /* Initialize Audio Recorder */
    if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, OUTPUT_DEVICE_HEADPHONE, I2S_AUDIOFREQ_16K, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) == AUDIO_OK)
    {
        BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"  AUDIO INIT OK  ", CENTER_MODE);
    }
    else
    {
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"  AUDIO RECORD INIT FAIL", CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, 60, (uint8_t *)" Try to reset board ", CENTER_MODE);
    }

    audio_block_size = nsamp * 8;
    navgs_ = 2;
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

    BSP_AUDIO_IN_SetVolume(DEFAULT_MIC_GAIN);

    BSP_TS_GetState(&prev_TS_State);
    radio_screen(&prev_TS_State);

    lora_rx_begin();
    currently_decoding = 0;
    rx_size = -1;

    while (1)
    {
        TS_StateTypeDef this_TS_State;

        BSP_TS_GetState(&this_TS_State);
        if (this_TS_State.touchDetected != prev_TS_State.touchDetected ||
            this_TS_State.touchX != prev_TS_State.touchY ||
            this_TS_State.touchY != prev_TS_State.touchY)
        {
            radio_screen(&this_TS_State);
            memcpy(&prev_TS_State, &this_TS_State, sizeof(TS_StateTypeDef));
        }

        if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_SET) {
            short *inPtr = NULL;
            if (user_button_pressed == 0) {
                user_button_pressed = 1;
                audio_rec_buffer_state = BUFFER_OFFSET_NONE;
                /* rx -> tx mode switch */
                printf("keyup\r\n");
                lorahal.standby();
                appHal.lcd_printOpMode(false);
                tx_buf_idx = 0;
                cycleStartAt = 0;
                mid = 0;
                frameCnt = 0;
            }

            if (audio_rec_buffer_state != BUFFER_OFFSET_NONE) {
                if (tx_buf_idx == 0 && mid == 0) {
                    cycleDur = uwTick - cycleStartAt;
                    cycleStartAt = uwTick;
                }

                if (audio_rec_buffer_state == BUFFER_OFFSET_FULL) {
                    inPtr = (short*)audio_buffer_in_B;
                } else if (audio_rec_buffer_state == BUFFER_OFFSET_HALF) {
                    inPtr = (short*)audio_buffer_in;
                }
                audio_rec_buffer_state = BUFFER_OFFSET_NONE;

                if (selected_bitrate == CODEC2_MODE_1300 || selected_bitrate == CODEC2_MODE_700C) {
                    unsigned i;
                    if (mid) {
                        unsigned oidx, iidx;
                        decimated_encode((short*)inPtr, scratch);
                        printf("B ");
                        for (i = 0; i <= frame_length_bytes; i++) {
                            printf("%02x ", scratch[i]);
                        }
                        printf("\r\n");
                        oidx = frame_length_bytes;
                        lorahal.tx_buf[tx_buf_idx+oidx] |= scratch[0] >> 4;
                        oidx++;
                        iidx = 0;
                        for (i = 0; i <= frame_length_bytes; i++) {
                            lorahal.tx_buf[tx_buf_idx+oidx] = scratch[iidx++] << 4;
                            lorahal.tx_buf[tx_buf_idx+oidx] |= scratch[iidx] >> 4;
                            oidx++;
                        }

                        printf("out ");
                        for (i = 0; i < _bytes_per_frame; i++)
                            printf("%02x ", lorahal.tx_buf[tx_buf_idx+i]);
                        printf("\r\n");
                        tx_buf_idx += _bytes_per_frame;
                        mid = 0;
                    /* ...second half */ } else { /* first half: */
                        lorahal.tx_buf[tx_buf_idx+frame_length_bytes] = 0;
                        decimated_encode((short*)inPtr, &lorahal.tx_buf[tx_buf_idx]);
                        printf("\r\nA ");
                        for (i = 0; i <= frame_length_bytes; i++)
                            printf("%02x ", lorahal.tx_buf[tx_buf_idx+i]);
                        printf("\r\n");
                        mid = 1;
                    }

                } else if (selected_bitrate == CODEC2_MODE_450 || selected_bitrate == CODEC2_MODE_450PWB) {
                    uint8_t o;
                    decimated_encode((short*)inPtr, scratch);
                    printf("%c) ", 'A' + mid);
                    for (unsigned i = 0; i <= frame_length_bytes; i++) {
                        printf("%02x ", scratch[i]);
                    }
                    printf("\r\n");
                    switch (mid) {  // {} = 18bit source
                        case 0:
                            idx_start = tx_buf_idx;
                            lorahal.tx_buf[tx_buf_idx++] = scratch[0];   // n=0 {0, 1, 2, 3, 4, 5, 6, 7}
                            lorahal.tx_buf[tx_buf_idx++] = scratch[1];   // n=1 {8, 9,10,11,12,13,14,15}
                            lorahal.tx_buf[tx_buf_idx] = scratch[2];     // n=2 {16,17}
                            mid = 1;
                            break;
                        case 1:
                            o = scratch[0] & 0xfc;                      // n=2 {0,1,2,3,4,5}
                            o >>= 2;
                            lorahal.tx_buf[tx_buf_idx++] |= o;
                            o = scratch[0] & 0x03;
                            o <<= 6;
                            lorahal.tx_buf[tx_buf_idx] = o;              // n=3 {6,7}
                            o = scratch[1] & 0xfc;
                            o >>= 2;
                            lorahal.tx_buf[tx_buf_idx++] |= o;            // n=3 {8,9,10,11,12,13}
                            o = scratch[1] & 0x03;                      // n=4 {14,15}
                            o <<= 6;
                            lorahal.tx_buf[tx_buf_idx] = o;
                            o = scratch[2];
                            o >>= 2;
                            lorahal.tx_buf[tx_buf_idx] |= o;             // n=4 {16,17}
                            mid = 2;
                            break;
                        case 2:
                            o = scratch[0] & 0xf0;                      // n=4 {0,1,2,3}
                            o >>= 4;
                            lorahal.tx_buf[tx_buf_idx++] |= o;
                            o = scratch[0] & 0x0f;                      // n=5 {4,5,6,7}
                            o <<= 4;
                            lorahal.tx_buf[tx_buf_idx] = o;
                            o = scratch[1] & 0xf0;                      // n=5 {8,9,10,11}
                            o >>= 4;
                            lorahal.tx_buf[tx_buf_idx++] |= o;
                            o = scratch[1] & 0x0f;                      // n=6 {12,13,14,15}
                            o <<= 4;
                            lorahal.tx_buf[tx_buf_idx] = o;
                            o = scratch[2];                             // n=6 {16,17}
                            o >>= 4;
                            lorahal.tx_buf[tx_buf_idx] |= o;
                            mid = 3;
                            break;
                        case 3:
                            o = scratch[0] & 0xc0;                      // n=6 {0,1}
                            o >>= 6;
                            lorahal.tx_buf[tx_buf_idx++] |= o;
                            o = scratch[0] & 0x3f;                      // n=7 {2,3,4,5,6,7}
                            o <<= 2;
                            lorahal.tx_buf[tx_buf_idx] = o;
                            o = scratch[1] & 0xc0;                      // n=7 {8,9}
                            o >>= 6;
                            lorahal.tx_buf[tx_buf_idx++] |= o;
                            o = scratch[1] & 0x3f;                      // n=8 {10,11,12 13,14,15}
                            o <<= 2;
                            lorahal.tx_buf[tx_buf_idx] = o;
                            o = scratch[2];                             // n=8 {16,17}
                            o >>= 6;
                            lorahal.tx_buf[tx_buf_idx++] |= o;
                            printf("out:");
                            for (unsigned n = idx_start; n < tx_buf_idx; n++) {
                                printf("%02x ", lorahal.tx_buf[n]);
                            }
                            printf("\r\n");
                            mid = 0;
                            break;
                    }
                } else {
                    decimated_encode((short*)inPtr, &lorahal.tx_buf[tx_buf_idx]);
                    tx_buf_idx += _bytes_per_frame;
                }

                lorahal.service();

                if (tx_buf_idx >= lora_payload_length) {
                    tx_encoded(tx_buf_idx);
                    tx_buf_idx = 0;
                    mid = 0;
                }

            } // ..if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
        } else {
            if (user_button_pressed == 1) {
                /* tx -> rx mode switch */
                user_button_pressed = 0;
                printf("unkey %s tx_buf_idx:%u\r\n", txing ? "txing" : "-", tx_buf_idx);
                if (txing) {
                    rx_start_at_tx_done = 1;
                } else if (tx_buf_idx > 0) {
                    lorahal.service();
                    tx_encoded(tx_buf_idx);
                    rx_start_at_tx_done = 1;
                } else
                    lora_rx_begin();
            }
        }

        svc_uart();
        lorahal.service();

        if (rx_size != -1) {
            parse_rx();
            rx_size = -1;
        }

        if (terminate_spkr_rx) {
            printf("terminate_spkr_rx ");
            end_rx_tone();
            terminate_spkr_rx = 0;
        }

    } // .. while(1)

} // ..AudioLoopback_demo()

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
