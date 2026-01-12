/**
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "string.h"
#include "radio.h"
#include "opus_wrapper.h"
#ifdef ENABLE_TEST_AUDIO
#include "test_audio_8000.h"
#include "test_audio_16000.h"
#include "test_audio_48000.h"
#endif
#ifdef ENABLE_LR20XX
#include "lr20xx.h"
#include "lr20xx_radio_fifo.h"
#endif
#ifdef USE_FREERTOS
#include "freertos_tasks.h"
#endif

extern struct OPUS_WRAPPER *ow;

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

/* Separate pending flags for multi-packet mode to avoid lost callbacks.
 * When both callbacks fire before main loop services them, the single
 * audio_play_state_ would lose one. These flags allow both to be pending. */
volatile uint8_t audio_half_pending;
volatile uint8_t audio_full_pending;

/* Multi-packet mode audio INPUT pending flags.
 * During TX, we may receive multiple audio input callbacks before we can
 * process them. These flags queue them so we don't lose audio data. */
volatile uint8_t audio_in_half_pending;
volatile uint8_t audio_in_full_pending;

volatile uint32_t terminate_spkr_at_tick;

/* DMA overrun detection */
volatile uint8_t dma_during_encode = 0;  /* set by DMA callback during encode */
uint32_t dma_overrun_count = 0;          /* count of DMA callbacks during encode */
uint32_t enc_time_max = 0;               /* max encode time in ms */

/* TX cycle time monitoring - detect when encode+TX exceeds frame period */
uint32_t last_encode_tick = 0;           /* timestamp of last encode start */
uint32_t tx_cycle_overrun_count = 0;     /* count of cycles exceeding frame period */

/* TX buffer write position - shared with radio_lr20xx.c for memmove sync */
volatile uint16_t tx_buf_idx = 0;
uint32_t tx_cycle_max = 0;               /* max cycle time in ms */

volatile uint8_t user_button_pressed;   // flag
volatile uint8_t txing;   // flag
volatile uint8_t rx_start_at_tx_done;   // flag
volatile uint8_t terminate_spkr_rx;   // flag
volatile uint8_t uart_tx_active;   // flag for UART-controlled TX

/* Test mode variables */
uint8_t test_mode;              // flag for sequence number test mode
#ifdef ENABLE_TEST_AUDIO
uint8_t audio_test_mode;        // flag for test audio TX mode
uint32_t audio_test_idx;        // index into test audio array (shared with freertos_tasks.c)
const int16_t *test_audio_ptr;  // pointer to test audio data
uint32_t test_audio_len;        // length of test audio data
#endif
uint8_t sine_playback_mode;     // flag for direct sine wave playback on speaker
static uint16_t sine_play_idx;  // sine table index for playback
static uint32_t audioRate_;     // actual I2S sample rate for sine step calculation
static uint16_t sine_step;      // sine table step for 1kHz at current sample rate (playback)
uint16_t encoder_sine_step;     // sine table step for 1kHz at Opus sample rate (encoding)
uint16_t tx_seq_num;            // transmitter sequence number
uint16_t expected_rx_seq;       // receiver expected sequence number
uint32_t rx_frame_count;        // total frames received
uint32_t rx_dropped_count;      // dropped frames detected
uint32_t rx_duplicate_count;    // duplicate frames detected
uint32_t rx_corrupt_count;      // corrupted frames detected (pattern mismatch)
uint32_t rx_gap_count;          // streaming gaps detected (exceeding inter_pkt_timeout)
uint32_t rx_overflow_count;     // packets with overflow data from next packet
uint32_t rx_overflow_bytes;     // total overflow bytes preserved
uint32_t last_rx_pkt_tick;      // timestamp of last received packet

#ifdef ENABLE_LR20XX
extern uint32_t get_fifo_rx_irq_count(void);
extern uint32_t get_fifo_rx_total_bytes(void);
extern uint16_t get_fifo_rx_max_idx(void);
extern void reset_fifo_rx_debug(void);
/* streaming_config_t and streaming_cfg already defined in radio.h (lr20xx_hal) */
#endif

/* For touchscreen throttling in high-rate modes */
extern uint8_t frames_per_sec;

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
    audio_in_full_pending = 1;  /* For multi-packet mode: queue callback */
    dma_during_encode = 1;  /* signal that DMA completed during processing */
#ifdef USE_FREERTOS
    freertos_audio_full_ready_FromISR();
#endif
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
    audio_rec_buffer_state = BUFFER_OFFSET_HALF;
    audio_in_half_pending = 1;  /* For multi-packet mode: queue callback */
    dma_during_encode = 1;  /* signal that DMA completed during processing */
#ifdef USE_FREERTOS
    freertos_audio_half_ready_FromISR();
#endif
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

static volatile uint32_t audio_callback_overrun;

/* Deferred overrun debug info - set in ISR, printed in main loop */
static volatile uint8_t overrun_debug_pending;
static volatile uint8_t overrun_debug_callback;  /* 'H' or 'F' */
static volatile uint8_t overrun_debug_prev_state;
static volatile uint8_t overrun_debug_half_pend;
static volatile uint8_t overrun_debug_full_pend;
static volatile uint32_t overrun_debug_tick;

void BSP_AUDIO_OUT_TransferComplete_CallBack()
{
    if (audio_play_state_ != AUDIO_PLAY_NONE) {
        audio_callback_overrun++;
        /* Capture debug info for deferred printing */
        overrun_debug_callback = 'F';
        overrun_debug_prev_state = audio_play_state_;
        overrun_debug_half_pend = audio_half_pending;
        overrun_debug_full_pend = audio_full_pending;
        overrun_debug_tick = uwTick;
        overrun_debug_pending = 1;
    }
    audio_play_state_ = AUDIO_PLAY_FULL;
    audio_full_pending = 1;  /* Separate flag for multi-packet mode */
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack()
{
    audio_half_pending = 1;  /* Separate flag for multi-packet mode */
    if (audio_play_state_ != AUDIO_PLAY_NONE) {
        audio_callback_overrun++;
        /* Capture debug info for deferred printing */
        overrun_debug_callback = 'H';
        overrun_debug_prev_state = audio_play_state_;
        overrun_debug_half_pend = audio_half_pending;
        overrun_debug_full_pend = audio_full_pending;
        overrun_debug_tick = uwTick;
        overrun_debug_pending = 1;
    }
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
    } else if (rxchar == 'R') {
        printf("Resetting MCU...\r\n");
        HAL_Delay(10);  /* allow printf to complete */
        NVIC_SystemReset();
    } else if (rxchar == 't') {
        uart_tx_active = 1;
        printf("TX start\r\n");
    } else if (rxchar == 'r') {
        uart_tx_active = 0;
        printf("TX end enc_max=%lu cycle_max=%lu overruns=%lu\r\n",
               enc_time_max, tx_cycle_max, tx_cycle_overrun_count);
        enc_time_max = 0;  /* Reset for next TX session */
        tx_cycle_max = 0;
        tx_cycle_overrun_count = 0;
        last_encode_tick = 0;
    } else if (rxchar == 'T') {
        test_mode ^= 1;
        tx_seq_num = 0;
        expected_rx_seq = 0;
        rx_frame_count = 0;
        rx_dropped_count = 0;
        rx_duplicate_count = 0;
        rx_corrupt_count = 0;
        rx_gap_count = 0;
        rx_overflow_count = 0;
        rx_overflow_bytes = 0;
        last_rx_pkt_tick = 0;
#ifdef ENABLE_LR20XX
        reset_fifo_rx_debug();
#endif
        printf("Test mode %s\r\n", test_mode ? "ON" : "OFF");
#ifdef ENABLE_TEST_AUDIO
    } else if (rxchar == 'A') {
        audio_test_mode ^= 1;
        audio_test_idx = 0;
        if (audio_test_mode) {
            /* Select test audio based on Opus sample rate */
            int opus_sr = opus_wrapper_get_sample_rate(ow);
            switch (opus_sr) {
                case 8000:
                    test_audio_ptr = test_audio_8000;
                    test_audio_len = TEST_AUDIO_8000_SAMPLES;
                    break;
                case 16000:
                    test_audio_ptr = test_audio_16000;
                    test_audio_len = TEST_AUDIO_16000_SAMPLES;
                    break;
                case 48000:
                default:
                    test_audio_ptr = test_audio_48000;
                    test_audio_len = TEST_AUDIO_48000_SAMPLES;
                    break;
            }
            printf("Audio test mode ON (voice @ %dHz, %lu samples)\r\n", opus_sr, test_audio_len);
        } else {
            printf("Audio test mode OFF\r\n");
        }
#endif
    } else if (rxchar == 'S') {
        /* Toggle local sine wave playback on speaker for audio reference */
        sine_playback_mode ^= 1;
        sine_play_idx = 0;
        if (!sine_playback_mode) {
            /* Clear audio buffers to stop DMA from replaying old sine data */
            memset(audio_buffer_out, 0, audio_block_size);
            memset(audio_buffer_out_B, 0, audio_block_size);
            SCB_CleanDCache_by_Addr((uint32_t*)audio_buffer_out, audio_block_size);
            SCB_CleanDCache_by_Addr((uint32_t*)audio_buffer_out_B, audio_block_size);
        }
        printf("Sine playback %s (direct to speaker)\r\n", sine_playback_mode ? "ON" : "OFF");
    } else if (rxchar == '?') {
        printf("Test stats: frames=%lu dropped=%lu dup=%lu corrupt=%lu gaps=%lu overflow=%lu/%lu\r\n",
               rx_frame_count, rx_dropped_count, rx_duplicate_count, rx_corrupt_count, rx_gap_count,
               rx_overflow_count, rx_overflow_bytes);
        printf("enc_max:%lu ms overruns:%lu (budget %dms)\r\n",
               enc_time_max, dma_overrun_count, 1000 / frames_per_sec);
#ifdef ENABLE_LR20XX
        printf("  fifo_rx_irqs=%lu total_bytes=%lu max_idx=%u\r\n",
               get_fifo_rx_irq_count(), get_fifo_rx_total_bytes(), get_fifo_rx_max_idx());
#endif
    } else if (rxchar == 'b' || rxchar == 'B') {
        /* Step bandwidth only, let adjust_sf_for_streaming() set optimal SF
         * 'b' = BW down, 'B' = BW up */
        bool up = (rxchar == 'B');
        /* Must be in standby to change modulation params (per datasheet) */
        lorahal.standby();
        if (up && !lorahal.bw_at_highest()) {
            appHal.step_bw(true);
        } else if (!up && !lorahal.bw_at_lowest()) {
            appHal.step_bw(false);
        } else {
            printf("BW at limit\r\n");
        }
#ifdef ENABLE_LR20XX
        /* Auto-adjust SF for streaming feasibility and recalculate timing */
        adjust_sf_for_streaming();
        calculate_streaming_config();
        printf("BW=%ukHz SF%u\r\n", get_bw_khz_lr20xx(), get_sf_lr20xx());
#endif
        /* Return to RX mode */
        lorahal.rx(0);
    } else if (rxchar == 'f' || rxchar == 'F') {
        /* Step SF only: 'f' = faster (SF down), 'F' = slower (SF up) */
        bool slower = (rxchar == 'F');
        /* Must be in standby to change modulation params (per datasheet) */
        lorahal.standby();
        if (slower && !lorahal.sf_at_slowest()) {
            appHal.step_sf(true);
        } else if (!slower && !lorahal.sf_at_fastest()) {
            appHal.step_sf(false);
        } else {
            printf("SF at limit\r\n");
        }
#ifdef ENABLE_LR20XX
        /* Check if SF is feasible for streaming and recalculate timing */
        adjust_sf_for_streaming();
        calculate_streaming_config();
        printf("BW=%ukHz SF%u\r\n", get_bw_khz_lr20xx(), get_sf_lr20xx());
#endif
        /* Return to RX mode */
        lorahal.rx(0);
    } else if (rxchar == 'c' || rxchar == 'C') {
        /* Opus complexity: 'c' = decrease, 'C' = increase */
        int current = opus_wrapper_get_complexity(ow);
        if (rxchar == 'C' && current < 10) {
            opus_wrapper_set_complexity(ow, current + 1);
        } else if (rxchar == 'c' && current > 0) {
            opus_wrapper_set_complexity(ow, current - 1);
        } else {
            printf("Complexity at limit\r\n");
        }
        printf("Opus complexity: %d\r\n", opus_wrapper_get_complexity(ow));
    } else {
        printf("Commands:\r\n");
        printf("  t: TX start    r: TX end (RX)\r\n");
        printf("  T: toggle test mode (seq nums)\r\n");
        printf("  S: toggle silence test mode\r\n");
        printf("  ?: test stats  .: radio status\r\n");
        printf("  q/a: mic vol   w/s: spkr vol\r\n");
        printf("  b/B: BW down/up (SF auto-adjusted)\r\n");
        printf("  f/F: SF down/up (faster/slower)\r\n");
        printf("  c/C: Opus complexity down/up\r\n");
        printf("  R: reset MCU\r\n");
    }
}

/* Fill frame with sequence number for test mode */
void test_mode_encode(uint8_t *out, uint16_t frame_size)
{
    unsigned i;
    /* Put 16-bit sequence number at start of frame */
    out[0] = tx_seq_num & 0xff;
    out[1] = (tx_seq_num >> 8) & 0xff;
    /* Fill rest with pattern based on seq num for verification */
    for (i = 2; i < frame_size; i++) {
        out[i] = (uint8_t)(tx_seq_num + i);
    }
    tx_seq_num++;
}

/* Check sequence number in received frame for test mode */
void test_mode_decode(const uint8_t *in)
{
    static uint32_t test_frame_cnt;
    static uint32_t test_tick_at_second;
    static uint32_t dropped_this_period;
    static uint32_t dup_this_period;
    static uint32_t corrupt_this_period;
    uint16_t rx_seq = in[0] | (in[1] << 8);

    rx_frame_count++;
    test_frame_cnt++;

    if (rx_seq == expected_rx_seq) {
        /* Frame received in order - verify full frame pattern */
        unsigned i;
        int corrupt = 0;
        for (i = 2; i < _bytes_per_frame; i++) {
            uint8_t expected = (uint8_t)(rx_seq + i);
            if (in[i] != expected) {
                if (!corrupt) {
                    printf("\e[35mCORRUPT seq=%u byte[%u]=%02x expect=%02x\e[0m\r\n",
                           rx_seq, i, in[i], expected);
                }
                corrupt = 1;
            }
        }
        if (corrupt) {
            rx_corrupt_count++;
            corrupt_this_period++;
        }
        expected_rx_seq++;
    } else if (rx_seq > expected_rx_seq) {
        /* Check for garbage - if jump is too large, it's likely noise/BUSY bytes */
        uint16_t gap = rx_seq - expected_rx_seq;
        if (gap > 1000) {
            /* Garbage frame (e.g., 0xF4F4 BUSY pattern) - ignore completely */
            rx_frame_count--;  /* Don't count this garbage frame */
            test_frame_cnt--;
            return;
        }
        /* Frames were dropped */
        rx_dropped_count += gap;
        dropped_this_period += gap;
        printf("\e[31mDROP %u frames (got %u, expected %u)\e[0m\r\n",
               gap, rx_seq, expected_rx_seq);
        expected_rx_seq = rx_seq + 1;
    } else {
        /* Duplicate or out-of-order frame */
        rx_duplicate_count++;
        dup_this_period++;
        printf("\e[33mDUP/OOO frame (got %u, expected %u)\e[0m\r\n",
               rx_seq, expected_rx_seq);
    }

    /* Periodic status every frames_per_sec frames */
    if (test_frame_cnt >= frames_per_sec) {
        uint32_t elapsed = uwTick - test_tick_at_second;
        if (dropped_this_period || dup_this_period || corrupt_this_period) {
            printf("%lums for %lu frames (\e[31m%lu dropped\e[0m, \e[33m%lu dup\e[0m, \e[35m%lu corrupt\e[0m)\r\n",
                   elapsed, test_frame_cnt, dropped_this_period, dup_this_period, corrupt_this_period);
        } else {
            printf("%lums for %lu frames OK\r\n", elapsed, test_frame_cnt);
        }
        test_tick_at_second = uwTick;
        test_frame_cnt = 0;
        dropped_this_period = 0;
        dup_this_period = 0;
        corrupt_this_period = 0;
    }
}

#define N_HISTORY       12
void decimated_encode(const short *in, uint8_t *out)
{
    short to_encoder[OPUS_WRAPPER_SAMPLES_PER_FRAME_MAX];  /* max size for 60ms @ 48kHz */
    unsigned i, x;
    int sum;

#ifdef ENABLE_TEST_AUDIO
    /* Audio test mode: use recorded voice sample instead of microphone */
    if (audio_test_mode && test_audio_ptr != NULL) {
        for (x = 0; x < nsamp; x++) {
            to_encoder[x] = test_audio_ptr[audio_test_idx];
            audio_test_idx++;
            if (audio_test_idx >= test_audio_len)
                audio_test_idx = 0;  /* loop */
        }
    } else
#endif
    {
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
    }

    {
        uint32_t t0 = uwTick;
        dma_during_encode = 0;  /* clear before encode */
        int enc_len = opus_wrapper_encode(ow, to_encoder, out);
        if (dma_during_encode) {
            dma_overrun_count++;
        }
        uint32_t dt = uwTick - t0;
        if (dt > enc_time_max) enc_time_max = dt;

        /* Validate encoder output: must be exactly _bytes_per_frame for CBR mode.
         * If encoder returns fewer bytes, pad with zeros to ensure fixed frame size.
         * This is critical for multi-packet modes (64K, 96K) where we don't send
         * per-frame length information. */
        if (enc_len < 0) {
            /* Encoder error - fill with zeros */
            memset(out, 0, _bytes_per_frame);
            static uint32_t enc_error_count;
            if (++enc_error_count <= 5)
                printf("\e[31mEncode error %d\e[0m\r\n", enc_len);
        } else if ((uint16_t)enc_len < _bytes_per_frame) {
            /* Encoder produced fewer bytes than expected - pad with zeros */
            memset(&out[enc_len], 0, _bytes_per_frame - enc_len);
            static uint32_t enc_short_count;
            if (++enc_short_count <= 5)
                printf("\e[33mEncode short: %d < %u\e[0m\r\n", enc_len, _bytes_per_frame);
        } else if ((uint16_t)enc_len > _bytes_per_frame) {
            /* Encoder produced more bytes than expected - this is a problem!
             * We'll only transmit _bytes_per_frame, truncating the Opus frame.
             * The decoder will fail on this truncated frame. */
            static uint32_t enc_long_count;
            if (++enc_long_count <= 5)
                printf("\e[31mEncode LONG: %d > %u (TRUNCATED!)\e[0m\r\n", enc_len, _bytes_per_frame);
        }
    }

    if (++frameCnt == frames_per_sec) {
        printf("%lums for %u frames\r\n", uwTick - tickAtFrameSecond, frames_per_sec);
        tickAtFrameSecond = uwTick;
        frameCnt = 0;
    }
}

void tx_encoded(uint16_t tx_nbytes)
{
    lorahal.service();

    if (txing) {
        printf("\e[31mstillTxing\e[0m\r\n");
        lcd_print_tx_duration(0, (unsigned)cycleDur);
    }

    txing = 1;
    txStartAt = uwTick;
#ifdef ENABLE_LR20XX
    /* Use streaming TX: send current produced data, FIFO callback sends rest */
    {
        uint16_t initial_bytes = tx_buf_produced;
        /* Round down to frame boundary */
        initial_bytes = (initial_bytes / _bytes_per_frame) * _bytes_per_frame;
        if (initial_bytes < _bytes_per_frame)
            initial_bytes = _bytes_per_frame;  /* minimum 1 frame */
        Send_lr20xx_streaming(tx_nbytes, initial_bytes);
    }
#else
    lorahal.send(tx_nbytes);
#endif
    appHal.lcd_printOpMode(true);
}

void lora_rx_begin()
{
#ifdef ENABLE_LR20XX
    extern volatile uint16_t rx_fifo_read_idx;
    extern volatile uint16_t rx_decode_idx;
    rx_fifo_read_idx = 0;
    rx_decode_idx = 0;
#endif
    lorahal.rx(0);
    appHal.lcd_printOpMode(false);
    frameCnt = 0;
}

void sine_out(unsigned skipcnt)
{
    short *outPtr = NULL;
    unsigned table_idx;
    AudioPlay_e state;

    skipcnt /= 2;

    __disable_irq();
    state = audio_play_state_;
    audio_play_state_ = AUDIO_PLAY_NONE;
    __enable_irq();

    if (state == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (state == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    table_idx = 0;
    {
        short *bufStart = outPtr;
        for (unsigned i = 0; i < audio_block_size; i++) {
            int16_t out = (int16_t)(sine_table[table_idx] - 0x8000);
            out /= 4;

            *outPtr++ = out;
            *outPtr++ = out;

            table_idx += skipcnt;
            if (table_idx > SINE_TABLE_LENGTH)
                table_idx -= SINE_TABLE_LENGTH;
        }
        SCB_CleanDCache_by_Addr((uint32_t*)bufStart, audio_block_size * 4);
    }
} // ..sine_out()

void start_tone()
{
    sine_out((rx_rssi+(rx_snr*4)) + 170);
}

void silence()
{
    short *outPtr = NULL;
    AudioPlay_e state;

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    __disable_irq();
    state = audio_play_state_;
    audio_play_state_ = AUDIO_PLAY_NONE;
    __enable_irq();

    if (state == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (state == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    memset(outPtr, 0, audio_block_size);
    SCB_CleanDCache_by_Addr((uint32_t*)outPtr, audio_block_size);

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    __disable_irq();
    state = audio_play_state_;
    audio_play_state_ = AUDIO_PLAY_NONE;
    __enable_irq();

    if (state == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (state == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    memset(outPtr, 0, audio_block_size);
    SCB_CleanDCache_by_Addr((uint32_t*)outPtr, audio_block_size);
} // ..silence()

void tone_out(uint8_t mask)
{
    short *outPtr = NULL;
    AudioPlay_e state;

    while (audio_play_state_ == AUDIO_PLAY_NONE)
        asm("nop");

    __disable_irq();
    state = audio_play_state_;
    audio_play_state_ = AUDIO_PLAY_NONE;
    __enable_irq();

    if (state == AUDIO_PLAY_FULL)
        outPtr = (short*)audio_buffer_out_B;
    else if (state == AUDIO_PLAY_HALF)
        outPtr = (short*)audio_buffer_out;

    {
        short *bufStart = outPtr;
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
        SCB_CleanDCache_by_Addr((uint32_t*)bufStart, audio_block_size * 4);
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

static volatile uint32_t put_spkr_half_count;
static volatile uint32_t put_spkr_full_count;
short *last_decoded_frame;  /* For feeding audio when no new decode ready */

static void fill_audio_buffer(short *outPtr);  /* Forward declaration */

/* Non-blocking: service pending audio callback with last decoded frame */
void service_audio_output(void)
{
    /* Sine playback mode works independently - doesn't need decoding active */
    if (!sine_playback_mode) {
        if (!currently_decoding)
            return;
        if (last_decoded_frame == NULL)
            return;
    }

    /* Use separate pending flags to avoid losing callbacks.
     * Only fill a buffer if it hasn't been filled with this frame yet.
     * This prevents the same frame from being written to both buffers,
     * which would cause audio repetition and clicks. */
    short *savedDecBuf = decBuf;
    if (!sine_playback_mode)
        decBuf = last_decoded_frame;

    /* Handle ONE pending callback per service call.
     * If both are pending, only fill the first one - the other will be filled
     * by the next call. Filling both with the same frame causes audio to repeat. */
    if (audio_half_pending) {
        audio_half_pending = 0;
        audio_play_state_ = AUDIO_PLAY_NONE;  /* Clear to prevent false overrun count */
        put_spkr_half_count++;
        fill_audio_buffer((short*)audio_buffer_out);
    } else if (audio_full_pending) {
        audio_full_pending = 0;
        audio_play_state_ = AUDIO_PLAY_NONE;  /* Clear to prevent false overrun count */
        put_spkr_full_count++;
        fill_audio_buffer((short*)audio_buffer_out_B);
    }

    decBuf = savedDecBuf;
}

static void fill_audio_buffer(short *outPtr)
{
    unsigned x;
    short *bufStart = outPtr;

    /* Sine playback mode: output 1kHz sine wave directly for reference */
    if (sine_playback_mode) {
        for (x = 0; x < nsamp * navgs_; x++) {
            int16_t sample = (int16_t)(sine_table[sine_play_idx] - 0x8000);
            sample /= 4;  /* reduce amplitude to match encoder input level */
            *outPtr++ = sample;  /* left */
            *outPtr++ = sample;  /* right */
            sine_play_idx = (sine_play_idx + sine_step) % SINE_TABLE_LENGTH;  /* 1kHz at any sample rate */
        }
        SCB_CleanDCache_by_Addr((uint32_t*)bufStart, nsamp * navgs_ * 4);
        return;
    }

    if (navgs_ == 1) {
        /* No upsampling - copy mono samples to stereo output */
        for (x = 0; x < nsamp; x++) {
            *outPtr++ = decBuf[x];
            *outPtr++ = decBuf[x];
        }
    } else {
        /* Upsampling with interpolation */
        for (x = 0; x < nsamp; x++) {
            short y0 = decBuf[x];
            short dy = 0;

            if (x < (nsamp - 1)) {
                dy = (decBuf[x+1] - y0) * step;
            }

            for (unsigned i = 0; i < navgs_; i++) {
                *outPtr++ = y0;
                *outPtr++ = y0;
                y0 += dy;
            }
        }
    }
    SCB_CleanDCache_by_Addr((uint32_t*)bufStart, nsamp * navgs_ * 4);
}

void put_spkr()
{
    /* Wait for audio DMA callback before filling buffer.
     * In multi-packet mode, frames are decoded faster than audio callbacks fire,
     * so we must block here to prevent frames from being lost.
     * Use sticky pending flags which persist until cleared - the blocking wait
     * on audio_play_state_ missed callbacks during Opus decode (~26ms). */

    /* Wait for at least one callback using pending flags */
    while (!audio_half_pending && !audio_full_pending)
        asm("nop");

    /* Handle ONE pending callback per decoded frame.
     * If both are pending, only fill the first one - the other will be filled
     * by the next decoded frame. Filling both with the same frame causes audio
     * to repeat (pitch sounds lower/slower). */
    if (audio_half_pending) {
        audio_half_pending = 0;
        audio_play_state_ = AUDIO_PLAY_NONE;  /* Clear to prevent false overrun count */
        put_spkr_half_count++;
        fill_audio_buffer((short*)audio_buffer_out);
    } else if (audio_full_pending) {
        audio_full_pending = 0;
        audio_play_state_ = AUDIO_PLAY_NONE;
        put_spkr_full_count++;
        fill_audio_buffer((short*)audio_buffer_out_B);
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
    static short from_decoderA[OPUS_WRAPPER_SAMPLES_PER_FRAME_MAX];  /* max size for 48kHz */
    static short from_decoderB[OPUS_WRAPPER_SAMPLES_PER_FRAME_MAX];

    if (test_mode) {
        /* Test mode: check sequence numbers, then decode for timing.
         * Also detect streaming gaps (time between packets > inter_pkt_timeout) */
        if (last_rx_pkt_tick != 0) {
            uint32_t gap = uwTick - last_rx_pkt_tick;
            if (gap > inter_pkt_timeout) {
                rx_gap_count++;
                printf("\e[35mGAP %lums\e[0m ", gap);
            }
        }
        last_rx_pkt_tick = uwTick;

        /* Debug: print received sequence numbers for multi-frame mode */
        if (streaming_cfg.frames_per_packet > 1) {
            static uint8_t rx_debug_cnt = 0;
            if (rx_debug_cnt < 5) {
                uint16_t f, seq;
                printf("RX_BUF:");
                for (f = 0; f < rx_size; f += _bytes_per_frame) {
                    seq = lorahal.rx_buf[f] | (lorahal.rx_buf[f + 1] << 8);
                    printf(" [%u]=%u", f, seq);
                }
                printf("\r\n");
                rx_debug_cnt++;
            }
        }
        for (n = 0; n < rx_size; n += _bytes_per_frame) {
            test_mode_decode(&lorahal.rx_buf[n]);
        }
        /* Fall through to decode path for realistic timing */
    }

    if (currently_decoding == 0) {
        fdb = 0;
        /* Clear decoder buffers to avoid garbage on first put_spkr() call.
         * When fdb=0, prevDecBuf=from_decoderB which would otherwise contain
         * stale/uninitialized data causing an audio click. */
        memset(from_decoderA, 0, sizeof(from_decoderA));
        memset(from_decoderB, 0, sizeof(from_decoderB));
        start_tone();
        currently_decoding = 1;
    } else {
        /* successive received packet */
        terminate_spkr_at_tick = 0;
    }

    /* Opus VBR: decode frames from rx buffer */
    for (n = 0; n < rx_size; n += _bytes_per_frame) {
        const uint8_t *encoded = &lorahal.rx_buf[n];
        decBuf = fdb ? from_decoderB : from_decoderA;
        prevDecBuf = fdb ? from_decoderA : from_decoderB;
        opus_wrapper_decode(ow, encoded, _bytes_per_frame, decBuf);
        put_spkr();
    }

    if (rx_size < lora_payload_length) {
        end_rx_tone();
    } else {
        terminate_spkr_at_tick = uwTick + inter_pkt_timeout;
    }
} // ..parse_rx()

#ifdef ENABLE_LR20XX
/* Streaming RX: decode frames as they arrive from FIFO */
void streaming_rx_decode(void)
{
    extern volatile uint16_t rx_fifo_read_idx;
    extern volatile uint16_t rx_decode_idx;
    static short from_decoderA[OPUS_WRAPPER_SAMPLES_PER_FRAME_MAX];  /* max size for 48kHz */
    static short from_decoderB[OPUS_WRAPPER_SAMPLES_PER_FRAME_MAX];

    /* Check if new frames are available */
    static uint8_t loop_debug_count = 0;
    while (rx_decode_idx + _bytes_per_frame <= rx_fifo_read_idx) {
        /* Don't decode past packet boundary (single-packet mode only).
         * In multi-packet mode, the while loop condition handles this.
         * In single-packet mode, limit to current packet to prevent
         * decoding overflow data before RX_DONE triggers. */
        if (streaming_cfg.packets_per_frame == 1) {
            uint16_t max_decode = (rx_size != -1) ? (uint16_t)rx_size : lora_payload_length;
            if (rx_decode_idx >= max_decode) {
                if (test_mode && loop_debug_count < 10) {
                    printf("DBG break: dec=%u >= max=%u (rx_size=%d)\r\n",
                           rx_decode_idx, max_decode, rx_size);
                    loop_debug_count++;
                }
                break;
            }
        }

        /* In test mode, check sequence numbers but still decode for timing */
        if (test_mode) {
            test_mode_decode(&lorahal.rx_buf[rx_decode_idx]);
            /* Fall through to decode path to maintain same timing as speech mode */
        }

        if (currently_decoding == 0) {
            fdb = 0;
            /* Clear decoder buffers to avoid garbage on first put_spkr() call.
             * When fdb=0, prevDecBuf=from_decoderB which would otherwise contain
             * stale/uninitialized data causing an audio click. */
            memset(from_decoderA, 0, sizeof(from_decoderA));
            memset(from_decoderB, 0, sizeof(from_decoderB));
            last_decoded_frame = from_decoderA;  /* Initialize for service_audio_output */
            audio_half_pending = 0;  /* Clear stale pending flags */
            audio_full_pending = 0;
            start_tone();
            currently_decoding = 1;
            if (!test_mode)
                printf("stream_start idx=%u\r\n", rx_decode_idx);
        }
        /* Reset timeout and clear any pending terminate flag since we have data */
        terminate_spkr_at_tick = 0;
        terminate_spkr_rx = 0;

        /* Opus VBR: decode 1 frame per _bytes_per_frame */
        {
            const uint8_t *encoded = &lorahal.rx_buf[rx_decode_idx];
            decBuf = fdb ? from_decoderB : from_decoderA;
            prevDecBuf = fdb ? from_decoderA : from_decoderB;

            int dec_samples = opus_wrapper_decode(ow, encoded, _bytes_per_frame, decBuf);
            /* Validate decoder output */
            if (dec_samples < 0) {
                /* Decoder error - fill with silence */
                memset(decBuf, 0, nsamp * sizeof(short));
            }

            last_decoded_frame = decBuf;  /* Save for service_audio_output() */
            put_spkr();
            /* DEC@ debug disabled to reduce main loop latency */
        }

        rx_decode_idx += _bytes_per_frame;
    }
    /* Debug: show why loop exited */
    if (test_mode && loop_debug_count < 10 && rx_decode_idx < lora_payload_length) {
        printf("DBG loop exit: dec=%u + bpf=%u > fifo=%u (expect %u)\r\n",
               rx_decode_idx, _bytes_per_frame, rx_fifo_read_idx, lora_payload_length);
        loop_debug_count++;
    }
}
#endif /* ENABLE_LR20XX */

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

        if (prevPressed == PRESSED_BW_DOWN && !lorahal.bw_at_lowest()) {
            appHal.step_bw(false);
            appHal.lcd_print_bw(0, y_base+52);
#ifdef ENABLE_LR20XX
            /* Auto-adjust SF for streaming feasibility and recalculate timing */
            adjust_sf_for_streaming();
            calculate_streaming_config();
            appHal.lcd_print_sf(x_sf_base, y_base+52);
#else
            appHal.step_sf(true); // keep same datarate, sf up
#endif
        } else if (prevPressed == PRESSED_BW_UP && !lorahal.bw_at_highest()) {
            appHal.step_bw(true);
            appHal.lcd_print_bw(0, y_base+52);
#ifdef ENABLE_LR20XX
            /* Auto-adjust SF for streaming feasibility and recalculate timing */
            adjust_sf_for_streaming();
            calculate_streaming_config();
            appHal.lcd_print_sf(x_sf_base, y_base+52);
#else
            appHal.step_sf(false);    // keep same datarate, sf down
#endif
        } else if (prevPressed == PRESSED_SF_DOWN) {
            appHal.step_sf(false);
            appHal.lcd_print_sf(x_sf_base, y_base+52);
#ifdef ENABLE_LR20XX
            adjust_sf_for_streaming();
            calculate_streaming_config();
#endif
        } else if (prevPressed == PRESSED_SF_UP) {
            appHal.step_sf(true);
            appHal.lcd_print_sf(x_sf_base, y_base+52);
#ifdef ENABLE_LR20XX
            adjust_sf_for_streaming();
            calculate_streaming_config();
#endif
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
    uint32_t audioRate;
    int opus_sr;

    /* Auto-select audio rate based on Opus mode sample rate */
    opus_sr = opus_wrapper_get_sample_rate(ow);
    switch (opus_sr) {
        case 8000:
            audioRate = I2S_AUDIOFREQ_8K;
            break;
        case 16000:
            audioRate = I2S_AUDIOFREQ_16K;
            break;
        case 48000:
        default:
            audioRate = I2S_AUDIOFREQ_48K;
            break;
    }

    /* Calculate decimation factor */
    navgs_ = audioRate / opus_sr;
    if (navgs_ < 1) navgs_ = 1;  /* no upsampling supported */
    audio_block_size = nsamp * navgs_ * 4;  /* stereo, 2 bytes per sample */
    step = 1.0 / navgs_;

    /* Store audio rate for sine wave step calculation */
    audioRate_ = audioRate;
    /* Calculate sine step for 1kHz: step = 256 * 1000 / sample_rate
     * Multiply by 4 to compensate for DMA buffer being 4x larger than one frame */
    sine_step = (256 * 1000 * 4) / audioRate;
    if (sine_step < 1) sine_step = 1;
    /* Encoder sine step is based on Opus sample rate (no 4x factor needed) */
    encoder_sine_step = (256 * 1000) / opus_sr;
    if (encoder_sine_step < 1) encoder_sine_step = 1;

    printf("audioRate=%lu, opus_sr=%d, navgs=%u, nsamp=%u, block=%u, complexity=%d, sine_step=%u\r\n",
           audioRate, opus_sr, navgs_, nsamp, audio_block_size, opus_wrapper_get_complexity(ow), sine_step);

    BSP_LCD_SetFont(&Font12);
    /* Initialize Audio Recorder */
    if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, OUTPUT_DEVICE_HEADPHONE, audioRate, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) == AUDIO_OK)
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

#ifdef USE_FREERTOS
    /* Enable concurrent encoding now that audio is initialized.
     * This allows the encoder task to run in parallel with TX. */
    freertos_enable_concurrent_encode();
#endif

    /* Touchscreen polling interval: longer for high-rate modes (64K/96K with 60ms frames)
     * to ensure lorahal.service() is called with lowest latency */
    uint32_t ts_poll_interval = (frames_per_sec < 20) ? 100 : 20;  /* ms */
    uint32_t last_ts_poll = 0;

    while (1)
    {
        TS_StateTypeDef this_TS_State;

        /* Throttle touchscreen polling to reduce latency for radio service */
        if ((uwTick - last_ts_poll) >= ts_poll_interval) {
            last_ts_poll = uwTick;
            BSP_TS_GetState(&this_TS_State);
            if (this_TS_State.touchDetected != prev_TS_State.touchDetected ||
                this_TS_State.touchX != prev_TS_State.touchY ||
                this_TS_State.touchY != prev_TS_State.touchY)
            {
                radio_screen(&this_TS_State);
                memcpy(&prev_TS_State, &this_TS_State, sizeof(TS_StateTypeDef));
            }
        }

        if ((BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_SET) || uart_tx_active) {
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
#ifdef ENABLE_LR20XX
                /* Reset state machine for new TX session */
                stream_state = STREAM_IDLE;
                tx_buf_produced = 0;
                audio_in_half_pending = 0;  /* Clear stale pending flags */
                audio_in_full_pending = 0;
                last_encode_tick = 0;       /* Reset cycle time measurement */
                tx_cycle_overrun_count = 0;
                tx_cycle_max = 0;
                if (streaming_cfg.packets_per_frame > 1) {
                    printf("keyup multi-pkt: bpf=%u, init=%u\r\n",
                           _bytes_per_frame, get_streaming_initial_bytes());
                }
#endif
#ifdef USE_FREERTOS
                /* Signal encoder task that TX session is starting */
                freertos_tx_session_start();
#endif
            }

            /* Use pending flags to avoid losing audio callbacks during TX.
             * Needed for both multi-packet mode (packets_per_frame > 1) and
             * multi-frame mode (frames_per_packet > 1) where TX time is significant.
             * If audio_rec_buffer_state was cleared but a callback fired, restore it. */
            if (streaming_cfg.packets_per_frame > 1 || streaming_cfg.frames_per_packet > 1) {
                if (audio_rec_buffer_state == BUFFER_OFFSET_NONE) {
                    /* Check pending flags in order: process HALF before FULL */
                    if (audio_in_half_pending) {
                        audio_rec_buffer_state = BUFFER_OFFSET_HALF;
                    } else if (audio_in_full_pending) {
                        audio_rec_buffer_state = BUFFER_OFFSET_FULL;
                    }
                }
            }

            if (audio_rec_buffer_state != BUFFER_OFFSET_NONE) {
#ifdef ENABLE_LR20XX
                /* State machine: handle WAIT_TX state - skip encoding, wait for TX to complete */
                if (stream_state == STREAM_WAIT_TX) {
                    /* For multi-packet mode, don't transition to STREAM_BUFFERING_NEXT until
                     * all packets are actually transmitted (handled by Radio_txDoneBottom).
                     * For normal mode, transition when FIFO is fully loaded. */
                    extern volatile uint16_t tx_bytes_sent;
                    if (streaming_cfg.packets_per_frame > 1) {
                        /* Multi-packet mode: wait for all packets to be transmitted */
                        if (tx_bytes_sent >= tx_total_size) {
                            /* All packets sent - start buffering next frame */
                            tx_buf_idx = 0;
                            mid = 0;
                            tx_buf_produced = 0;
                            stream_state = STREAM_BUFFERING_NEXT;
                        } else {
#ifdef USE_FREERTOS
                            if (freertos_concurrent_encode_enabled()) {
                                /* Concurrent mode: continue getting frames from encoder
                                 * task during TX. This allows pipelined encoding.
                                 * But only if buffer has space for another frame. */
                                if (tx_buf_idx + _bytes_per_frame <= LR20XX_BUF_SIZE) {
                                    /* Fall through to get encoded frames. */
                                } else {
                                    /* Buffer full - wait for TX to complete */
                                    lorahal.service();
                                    goto skip_encode;
                                }
                            } else
#endif
                            {
                                /* Non-concurrent mode: wait for TX to complete */
                                lorahal.service();
                                goto skip_encode;
                            }
                        }
                    } else if (tx_fifo_idx >= tx_total_size) {
                        /* Normal mode: FIFO fully loaded - start buffering next packet */
                        stream_state = STREAM_BUFFERING_NEXT;
                        /* DON'T reset tx_buf_idx - allow pipelined encoding to continue
                         * at current offset. Frames encoded during TX will be preserved
                         * and moved to buffer start when TxDone fires. */
                        /* Fall through to encode for next packet */
                    } else if (streaming_cfg.frames_per_packet > 1) {
                        /* Multi-frame mode: allow pipelined encoding during FIFO load.
                         * Continue encoding at current tx_buf_idx (beyond 240).
                         * Transition to BUFFERING_NEXT to allow encoding to continue. */
                        stream_state = STREAM_BUFFERING_NEXT;
                        /* Fall through to encode */
                    } else {
                        /* Single-frame mode: wait for FIFO to load. */
#ifdef USE_FREERTOS
                        if (freertos_concurrent_encode_enabled()) {
                            /* In concurrent mode, continue getting frames from encoder
                             * task but stay in STREAM_WAIT_TX so Send_lr20xx_fifo_continue()
                             * can still feed the FIFO. Don't transition to BUFFERING_NEXT
                             * until FIFO is fully loaded (handled by tx_fifo_idx check above).
                             * Only fall through if buffer has space. */
                            if (tx_buf_idx + _bytes_per_frame <= LR20XX_BUF_SIZE) {
                                /* Fall through to get encoded frames. */
                            } else {
                                /* Buffer full - wait for TX to complete */
                                lorahal.service();
                                goto skip_encode;
                            }
                        } else
#endif
                        {
                            lorahal.service();
                            goto skip_encode;
                        }
                    }
                }

                /* State machine: IDLE with non-zero tx_buf_idx means TxDone just fired
                 * Reset indices BEFORE encoding to avoid buffer overflow.
                 * For pipelined modes, preserve any buffered data that wasn't transmitted
                 * (frames encoded during TX that need to go in the next packet). */
                if (stream_state == STREAM_IDLE && tx_buf_idx > 0) {
                    if (tx_buf_idx > tx_total_size) {
                        /* Pipelined encoding: preserve untransmitted data.
                         * tx_total_size is the bytes just transmitted.
                         * Move remaining data to start of buffer.
                         * Works for both multi-packet and multi-frame modes. */
                        uint16_t remaining = tx_buf_idx - tx_total_size;
                        memmove(LR20xx_tx_buf, &LR20xx_tx_buf[tx_total_size], remaining);
                        tx_buf_idx = remaining;
                        tx_buf_produced = remaining;
                        mid = 0;  /* Reset mid since buffer positions changed */

                        /* If we have enough data, start TX immediately (no gap) */
                        if (remaining >= get_streaming_initial_bytes()) {
                            uint16_t tx_size = (streaming_cfg.packets_per_frame > 1) ?
                                               _bytes_per_frame : lora_payload_length;
                            /* Debug: show what we're about to transmit */
                            if (test_mode) {
                                uint16_t tx_seq = LR20xx_tx_buf[0] | (LR20xx_tx_buf[1] << 8);
                                static uint8_t idle_tx_dbg = 0;
                                if (idle_tx_dbg < 10) {
                                    printf("IDLE_TX: seq=%u rem=%u\r\n", tx_seq, remaining);
                                    idle_tx_dbg++;
                                }
                            }
                            tx_encoded(tx_size);
                            stream_state = STREAM_TX_ACTIVE;
                            /* Transition to WAIT_TX if we've filled the TX quota */
                            if (tx_buf_idx >= tx_size) {
                                stream_state = STREAM_WAIT_TX;
                            }
                        } else {
                            stream_state = STREAM_ENCODING;
                        }
                    } else {
                        tx_buf_idx = 0;
                        mid = 0;
                        stream_state = STREAM_ENCODING;
                    }
                    cycleDur = uwTick - cycleStartAt;
                    cycleStartAt = uwTick;
                }

                /* State machine: transition IDLE -> ENCODING on new cycle (fresh start) */
                if (mid == 0 && stream_state == STREAM_IDLE) {
                    stream_state = STREAM_ENCODING;
                    tx_buf_produced = 0;
                    cycleDur = uwTick - cycleStartAt;
                    cycleStartAt = uwTick;
                }
#else
                if (tx_buf_idx == 0 && mid == 0) {
                    cycleDur = uwTick - cycleStartAt;
                    cycleStartAt = uwTick;
                }
#endif

                if (audio_rec_buffer_state == BUFFER_OFFSET_FULL) {
                    inPtr = (short*)audio_buffer_in_B;
                    audio_in_full_pending = 0;  /* Clear pending flag */
                } else if (audio_rec_buffer_state == BUFFER_OFFSET_HALF) {
                    inPtr = (short*)audio_buffer_in;
                    audio_in_half_pending = 0;  /* Clear pending flag */
                }
#ifdef USE_FREERTOS
                /* In concurrent mode, don't clear audio_rec_buffer_state yet.
                 * We need it to remain set until we successfully get the encoded frame
                 * from the encoder task. Clear it after successful frame retrieval. */
                if (!freertos_concurrent_encode_enabled())
#endif
                    audio_rec_buffer_state = BUFFER_OFFSET_NONE;

                /* TX cycle time monitoring: measure time between encodes.
                 * If cycle time > frame period, we're falling behind real-time. */
                {
                    uint32_t now = uwTick;
                    if (last_encode_tick != 0) {
                        uint32_t cycle_time = now - last_encode_tick;
                        uint32_t frame_period = opus_wrapper_get_frame_ms(ow);
                        if (cycle_time > tx_cycle_max) {
                            tx_cycle_max = cycle_time;
                        }
                        /* Warn if cycle time exceeds frame period by more than 2ms */
                        if (cycle_time > frame_period + 2) {
                            tx_cycle_overrun_count++;
                            printf("\e[31mTX_SLOW: cycle=%lums > frame=%lums\e[0m\r\n",
                                   cycle_time, frame_period);
                        }
                    }
                    last_encode_tick = now;
                }

                /* Bounds check: prevent buffer overflow if encoding outpaces TX.
                 * This can happen in single-frame mode during BUFFERING_NEXT state. */
                if (tx_buf_idx + _bytes_per_frame > LR20XX_BUF_SIZE) {
                    static uint8_t overflow_warn_cnt = 0;
                    if (overflow_warn_cnt < 5) {
                        printf("\e[31mTX_BUF overflow prevented: idx=%u + bpf=%u > %u\e[0m\r\n",
                               tx_buf_idx, _bytes_per_frame, LR20XX_BUF_SIZE);
                        overflow_warn_cnt++;
                    }
                    /* CRITICAL: Clear audio_rec_buffer_state to prevent infinite loop.
                     * Without this, the same audio callback triggers repeatedly. */
                    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
                    lorahal.service();
                    goto skip_encode;
                }

                /* Opus encode: one frame per callback */
#ifdef USE_FREERTOS
                /* In concurrent mode, get encoded frame from encoder task instead of encoding here.
                 * This allows encoding to happen in parallel with TX transmission. */
                if (freertos_concurrent_encode_enabled()) {
                    /* Get encoded frame from encoder task */
                    int32_t enc_len = freertos_get_encoded_frame(&lorahal.tx_buf[tx_buf_idx], _bytes_per_frame);
                    if (enc_len <= 0) {
                        /* No encoded frame ready yet - wait for encoder task.
                         * Don't clear audio_rec_buffer_state so we retry on next loop.
                         * IMPORTANT: yield to let encoder task run (it has lower priority) */
                        static uint32_t wait_count = 0;
                        if ((++wait_count % 100) == 1) {
                            printf("MAIN: waiting for encoder (audio_state=%d)\r\n", audio_rec_buffer_state);
                        }

                        /* CRITICAL: Check if we have buffered data ready to send.
                         * Without this, TX gaps occur when encoder is slow (loud audio).
                         * Check both IDLE (just after TxDone) and ENCODING (normal state). */
                        if ((stream_state == STREAM_IDLE || stream_state == STREAM_ENCODING) &&
                            tx_buf_idx >= get_streaming_initial_bytes()) {
                            uint16_t tx_size = (streaming_cfg.packets_per_frame > 1) ?
                                               _bytes_per_frame : lora_payload_length;
                            tx_encoded(tx_size);
                            stream_state = STREAM_TX_ACTIVE;
                            if (tx_buf_idx >= tx_size) {
                                stream_state = STREAM_WAIT_TX;
                            }
                        }

                        lorahal.service();
                        vTaskDelay(1);  /* Yield to encoder task */
                        goto skip_encode;
                    }
                    /* Frame obtained from encoder task - now clear audio_rec_buffer_state */
                    static uint32_t got_count = 0;
                    if (++got_count <= 3) {
                        printf("MAIN: got frame %lu len=%ld\r\n", got_count, enc_len);
                    }
                    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
                } else
#endif
                {
                    /* Non-concurrent mode: encode directly in main loop */
                    decimated_encode((short*)inPtr, &lorahal.tx_buf[tx_buf_idx]);
                }

                if (test_mode) {
                    /* Test mode: overwrite encoded data with sequence numbers.
                     * Opus encode still runs above for realistic timing. */
                    test_mode_encode(&lorahal.tx_buf[tx_buf_idx], _bytes_per_frame);

                    /* Debug: verify byte 288 is written correctly for 64K mode */
                    if (_bytes_per_frame == 480) {
                        static uint8_t byte288_debug_cnt = 0;
                        uint8_t expect_288 = (uint8_t)(tx_seq_num - 1 + 288);  /* tx_seq_num was already incremented */
                        uint8_t actual_288 = lorahal.tx_buf[tx_buf_idx + 288];
                        if (byte288_debug_cnt < 3) {
                            printf("TX byte[288]=%02x expect=%02x at idx=%u\r\n",
                                   actual_288, expect_288, tx_buf_idx + 288);
                            byte288_debug_cnt++;
                        }
                    }
                    /* Debug: verify each frame for 16K mode */
                    if (streaming_cfg.frames_per_packet > 1) {
                        static uint8_t enc_debug_cnt = 0;
                        if (enc_debug_cnt < 12) {
                            uint16_t written_seq = lorahal.tx_buf[tx_buf_idx] |
                                                   (lorahal.tx_buf[tx_buf_idx + 1] << 8);
                            printf("ENC: idx=%u seq=%u state=%d\r\n",
                                   tx_buf_idx, written_seq, stream_state);
                            enc_debug_cnt++;
                        }
                    }
                }
                tx_buf_idx += _bytes_per_frame;

                lorahal.service();

#ifdef ENABLE_LR20XX
                /* Update producer index so FIFO callback can send new data */
                tx_buf_produced = tx_buf_idx;

                /* State machine: ENCODING -> TX_ACTIVE when enough data */
                if (stream_state == STREAM_ENCODING &&
                    tx_buf_idx >= get_streaming_initial_bytes()) {
                    /* For multi-packet mode (64K, 96K), send full Opus frame
                     * which spans multiple LoRa packets. For normal mode, send
                     * one LoRa packet containing multiple frames. */
                    uint16_t tx_size = (streaming_cfg.packets_per_frame > 1) ?
                                       _bytes_per_frame : lora_payload_length;
                    /* Debug: show initial TX start */
                    if (test_mode) {
                        uint16_t tx_seq = LR20xx_tx_buf[0] | (LR20xx_tx_buf[1] << 8);
                        static uint8_t init_tx_dbg = 0;
                        if (init_tx_dbg < 10) {
                            printf("INIT_TX: seq=%u idx=%u\r\n", tx_seq, tx_buf_idx);
                            init_tx_dbg++;
                        }
                    }
                    if (streaming_cfg.packets_per_frame > 1) {
                        static uint8_t tx_start_debug = 0;
                        if (tx_start_debug < 3) {
                            printf("tx_start: idx=%u size=%u\r\n", tx_buf_idx, tx_size);
                            tx_start_debug++;
                        }
                    }
                    /* Debug: verify sequence numbers in buffer for multi-frame mode */
                    if (test_mode && streaming_cfg.frames_per_packet > 1) {
                        static uint8_t mf_debug_cnt = 0;
                        if (mf_debug_cnt < 5) {
                            uint16_t f, seq;
                            printf("TX_BUF:");
                            for (f = 0; f < streaming_cfg.frames_per_packet; f++) {
                                seq = lorahal.tx_buf[f * _bytes_per_frame] |
                                      (lorahal.tx_buf[f * _bytes_per_frame + 1] << 8);
                                printf(" [%u]=%u", f * _bytes_per_frame, seq);
                            }
                            printf("\r\n");
                            mf_debug_cnt++;
                        }
                    }
                    tx_encoded(tx_size);
                    stream_state = STREAM_TX_ACTIVE;
                }

                /* State machine: TX_ACTIVE -> WAIT_TX when all frames encoded */
                {
                    uint16_t wait_threshold = (streaming_cfg.packets_per_frame > 1) ?
                                              _bytes_per_frame : lora_payload_length;
                    if (stream_state == STREAM_TX_ACTIVE &&
                        tx_buf_idx >= wait_threshold) {
                        stream_state = STREAM_WAIT_TX;
                        /* Don't reset tx_buf_idx - STREAM_WAIT_TX will check FIFO
                         * and transition to STREAM_BUFFERING_NEXT when ready */
                    }
                }

                /* STREAM_BUFFERING_NEXT: continue encoding for next packet.
                 * We stay in this state until TxDone fires (handled in radio_lr20xx.c).
                 * No state transition needed here - just keep filling the buffer.
                 * When tx_buf_idx reaches the threshold, we have a full packet
                 * ready and will start TX immediately when current TX completes. */
#else
                if (tx_buf_idx >= lora_payload_length) {
                    tx_encoded(tx_buf_idx);
                    tx_buf_idx = 0;
                    mid = 0;
                }
#endif

            } // ..if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
#ifdef ENABLE_LR20XX
skip_encode:
            (void)0;  /* Empty statement for label */
#endif
        } else {
            if (user_button_pressed == 1) {
                /* tx -> rx mode switch */
                user_button_pressed = 0;
                printf("unkey %s tx_buf_idx:%u\r\n", txing ? "txing" : "-", tx_buf_idx);
#ifdef ENABLE_LR20XX
                /* Stop buffering next packet - we're done transmitting */
                stream_state = STREAM_IDLE;
#endif
#ifdef USE_FREERTOS
                /* Signal encoder task that TX session is ending */
                freertos_tx_session_end();
#endif
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

#ifdef ENABLE_LR20XX
        /* Debug: periodic heartbeat in multi-packet mode */
        if (streaming_cfg.packets_per_frame > 1) {
            static uint32_t last_heartbeat = 0;
            static uint8_t heartbeat_count = 0;
            if (uwTick - last_heartbeat >= 500 && heartbeat_count < 10) {
                printf("HB: st=%d txing=%d buf=%u rec=%d\r\n",
                       stream_state, txing, tx_buf_idx, audio_rec_buffer_state);
                last_heartbeat = uwTick;
                heartbeat_count++;
            }
        }

        /* Service audio output for:
         * - Multi-packet mode: handle callbacks while waiting for packets
         * - Sine playback mode: standalone sine wave output for testing */
        if (streaming_cfg.packets_per_frame > 1 || sine_playback_mode) {
            service_audio_output();

            /* Clear overrun debug flag - count is reported at TX end.
             * Don't print here: during TX it adds latency, during idle it's noise. */
            overrun_debug_pending = 0;
        }

        /* Streaming RX: decode frames as they arrive from FIFO.
         * IMPORTANT: Do this BEFORE checking overflow, so we decode any valid
         * data that was read before the overflow occurred. */
        streaming_rx_decode();

        /* Check for FIFO overflow (deferred from IRQ) */
        {
            extern volatile uint8_t fifo_rx_overflow;
            extern volatile uint16_t rx_fifo_read_idx;
            extern volatile uint16_t rx_decode_idx;
            if (fifo_rx_overflow) {
                fifo_rx_overflow = 0;
                /* FIFO overflow means we lost bytes and alignment is corrupted.
                 * Clear hardware FIFO, software buffer, and reset indices to start fresh.
                 * Disable IRQs to prevent race with FIFO callback. */
                __disable_irq();
                lr20xx_radio_fifo_clear_rx(NULL);
                /* Clear software buffer to prevent garbage decodes */
                uint16_t clear_size = (streaming_cfg.packets_per_frame > 1) ? _bytes_per_frame : lora_payload_length;
                memset(LR20xx_rx_buf, 0, clear_size);
                rx_fifo_read_idx = 0;
                rx_decode_idx = 0;
                __enable_irq();
                static uint32_t overflow_print_cnt = 0;
                if (++overflow_print_cnt <= 5) {
                    printf("\e[31mRX FIFO overflow (reset)\e[0m\r\n");
                }
            }
        }
        /* Check for deferred FIFO residual warning */
        {
            extern volatile uint16_t rx_fifo_residual_bytes;
            uint16_t residual = rx_fifo_residual_bytes;
            if (residual > 0) {
                rx_fifo_residual_bytes = 0;
                /* WARN printf disabled to reduce main loop latency */
            }
        }
#endif

        if (rx_size != -1) {
#ifdef ENABLE_LR20XX
            /* Packet complete - handle single vs multi-packet modes */
            extern volatile uint16_t rx_fifo_read_idx;
            extern volatile uint16_t rx_decode_idx;
            uint8_t multi_packet_mode = (streaming_cfg.packets_per_frame > 1);
            uint16_t pkt_size = (uint16_t)rx_size;

            /* Check if FIFO has data - would indicate new packet started arriving */
            uint16_t fifo_level;
            if (lr20xx_radio_fifo_get_rx_level(NULL, &fifo_level) == LR20XX_STATUS_OK && fifo_level > 0) {
                /* WARN printf disabled to reduce main loop latency */
            }

            if (multi_packet_mode) {
                /* Multi-packet mode (64K, 96K): accumulate data until we have complete Opus frame */
                if (rx_fifo_read_idx >= _bytes_per_frame) {
                    /* Have enough data for at least one Opus frame - decode it */
                    streaming_rx_decode();

                    /* Move any remaining data to beginning of buffer */
                    __disable_irq();
                    uint16_t decoded = rx_decode_idx;
                    uint16_t remaining = rx_fifo_read_idx - decoded;
                    if (remaining > 0 && decoded > 0) {
                        memmove(LR20xx_rx_buf, &LR20xx_rx_buf[decoded], remaining);
                    }
                    /* Clear stale data beyond remaining to prevent garbage decodes */
                    if (remaining < _bytes_per_frame) {
                        memset(&LR20xx_rx_buf[remaining], 0, _bytes_per_frame - remaining);
                    }
                    rx_fifo_read_idx = remaining;
                    rx_decode_idx = 0;
                    __enable_irq();

                    /* Update timeout */
                    if (currently_decoding) {
                        terminate_spkr_at_tick = uwTick + inter_pkt_timeout;
                    }
                } else {
                    /* Still accumulating packets - extend timeout to wait for more.
                     * For multi-packet mode, we need to allow time for all packets
                     * of one Opus frame to arrive before timing out. */
                    terminate_spkr_at_tick = uwTick + inter_pkt_timeout;
                }
            } else {
                /* Normal single-packet mode */
                uint16_t saved_fifo_idx = rx_fifo_read_idx;
                uint16_t pre_decode_idx = rx_decode_idx;
                if (rx_fifo_read_idx > pkt_size) {
                    /* Limit decode to current packet only */
                    rx_fifo_read_idx = pkt_size;
                }
                streaming_rx_decode();
                uint16_t frames_decoded = (rx_decode_idx - pre_decode_idx) / _bytes_per_frame;
                /* Debug: show decode stats for first few packets */
                static uint8_t debug_pkt_count = 0;
                if (test_mode && debug_pkt_count < 5) {
                    printf("DBG pkt: size=%d fifo=%u->%u dec=%u->%u (%u frames)\r\n",
                           pkt_size, saved_fifo_idx, rx_fifo_read_idx,
                           pre_decode_idx, rx_decode_idx, frames_decoded);
                    debug_pkt_count++;
                }
                rx_fifo_read_idx = saved_fifo_idx;  /* Restore for overflow calculation */

                /* Set end-of-packet timeout */
                if (rx_size < lora_payload_length) {
                    if (!test_mode)
                        printf("short_pkt %d\r\n", rx_size);
                    end_rx_tone();
                } else if (currently_decoding) {
                    if (!test_mode) {
                        static uint32_t pkt_done_cnt = 0;
                        if (++pkt_done_cnt <= 10 || (pkt_done_cnt % 25) == 0) {
                            printf("pkt_done pkt=%u fifo=%u dec=%u\r\n", pkt_size, saved_fifo_idx, rx_decode_idx);
                        }
                    }
                    terminate_spkr_at_tick = uwTick + inter_pkt_timeout;
                }

                /* Preserve overflow data from next packet if present.
                 * The FIFO callback may have read data from the next packet that started
                 * arriving before we finished processing this one. */
                __disable_irq();
                uint16_t overflow = (saved_fifo_idx > pkt_size) ? (saved_fifo_idx - pkt_size) : 0;
                if (overflow > 0) {
                    /* Move overflow data to beginning of buffer */
                    memmove(LR20xx_rx_buf, &LR20xx_rx_buf[pkt_size], overflow);
                    memset(&LR20xx_rx_buf[overflow], 0, pkt_size - overflow);
                    rx_fifo_read_idx = overflow;
                    rx_overflow_count++;
                    rx_overflow_bytes += overflow;
                } else {
                    /* No overflow - just reset index */
                    rx_fifo_read_idx = 0;
                }
                rx_decode_idx = 0;
                __enable_irq();
            }
#else
            parse_rx();
#endif
            rx_size = -1;
        }

        if (terminate_spkr_rx) {
            extern volatile uint16_t rx_fifo_read_idx;
            extern volatile uint16_t rx_decode_idx;
            extern volatile uint32_t audio_callback_overrun;
            extern volatile uint32_t put_spkr_half_count;
            extern volatile uint32_t put_spkr_full_count;
            printf("terminate: fifo=%u dec=%u overrun=%lu half=%lu full=%lu\r\n",
                   rx_fifo_read_idx, rx_decode_idx, audio_callback_overrun,
                   put_spkr_half_count, put_spkr_full_count);
            audio_callback_overrun = 0;
            put_spkr_half_count = 0;
            put_spkr_full_count = 0;
            /* Clear hardware FIFO and indices to prevent garbage reads after TX ends.
             * The 0xF4 BUSY status byte can be read if FIFO is accessed when chip not ready. */
            __disable_irq();
            lr20xx_radio_fifo_clear_rx(NULL);
            uint16_t clear_size = (streaming_cfg.packets_per_frame > 1) ? _bytes_per_frame : lora_payload_length;
            memset(LR20xx_rx_buf, 0, clear_size);
            rx_fifo_read_idx = 0;
            rx_decode_idx = 0;
            __enable_irq();
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
