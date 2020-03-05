/**
  ******************************************************************************
  * @file    BSP/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_sd.h"
#include "stm32746g_discovery_eeprom.h"
#include "stm32746g_discovery_camera.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_qspi.h"
#include "codec2/src/codec2.h"

/* Macros --------------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT
/* Assert activated */
#define ASSERT(__condition__)                do { if(__condition__) \
                                                   {  assert_failed(__FILE__, __LINE__); \
                                                      while(1);  \
                                                    } \
                                              }while(0)
#else
/* Assert not activated : macro has no effect */
#define ASSERT(__condition__)                  do { if(__condition__) \
                                                   {  ErrorCounter++; \
                                                    } \
                                              }while(0)
#endif /* USE_FULL_ASSERT */

#define RGB565_BYTE_PER_PIXEL     2
#define ARBG8888_BYTE_PER_PIXEL   4

/* Camera have a max resolution of VGA : 640x480 */
#define CAMERA_RES_MAX_X          640
#define CAMERA_RES_MAX_Y          480

/**
  * @brief  LCD FB_StartAddress
  * LCD Frame buffer start address : starts at beginning of SDRAM
  */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

/**
  * @brief  Camera frame buffer start address
  * Assuming LCD frame buffer is of size 480x800 and format ARGB8888 (32 bits per pixel).
  */
#define CAMERA_FRAME_BUFFER       ((uint32_t)(LCD_FRAME_BUFFER + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

/**
  * @brief  SDRAM Write read buffer start address after CAM Frame buffer
  * Assuming Camera frame buffer is of size 640x480 and format RGB565 (16 bits per pixel).
  */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(CAMERA_FRAME_BUFFER + (CAMERA_RES_MAX_X * CAMERA_RES_MAX_Y * RGB565_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR

/* The Audio file is flashed with ST-Link Utility @ flash address =  AUDIO_SRC_FILE_ADDRESS */
#define AUDIO_SRC_FILE_ADDRESS       0x08080000   /* Audio file address in flash */

/* Exported types ------------------------------------------------------------*/

typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;

extern const unsigned char stlogo[];
/* Exported variables ---------------------------------------------------*/
extern uint8_t     NbLoop;
extern uint8_t     MfxExtiReceived;
#ifndef USE_FULL_ASSERT
extern uint32_t    ErrorCounter;
#endif
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void AudioLoopback_demo (void);
uint8_t AUDIO_Process(void);
uint8_t CheckForUserInput(void);
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#endif
extern UART_HandleTypeDef UartHandle;
extern volatile uint8_t uartReceived;
extern uint8_t rxchar;

extern struct CODEC2 *c2;
extern unsigned nsamp;
extern unsigned nsamp_x2;
#endif /* __MAIN_H */

extern uint8_t lora_payload_length;
extern uint8_t _bytes_per_frame;
extern uint8_t frame_length_bytes;
extern uint8_t sf_at_500KHz;

extern volatile uint8_t txing;   // flag
extern volatile uint8_t rx_start_at_tx_done;   // flag

extern volatile int rx_size;
extern volatile float rx_rssi;
extern volatile float rx_snr;
extern volatile uint32_t txStartAt;

extern unsigned inter_pkt_timeout;
extern int8_t selected_bitrate;  // upon touch release

//void UART_DMA_Init(COM_TypeDef COM);
void lcd_print_rssi_snr(float rssi, float snr);
void lcd_print_tx_duration(int dur, unsigned interval);
void rxDoneCB(uint8_t size, float rssi, float snr);
void lora_rx_begin(void);

/* radio.c: */
void start_radio(void);
bool sf_at_slowest(void);
bool sf_at_fastest(void);
bool bw_at_highest(void);
bool bw_at_lowest(void);

/* app_sx126x.c: */
typedef struct {
    void (*lcd_printOpMode)(bool);
    void (*radioPrintStatus)(void);
    void (*lcd_print_bw)(uint8_t, uint8_t);
    void (*lcd_print_sf)(uint8_t, uint8_t);
    void (*step_bw)(bool);
    void (*step_sf)(bool);
} loraAppHal_t;
extern loraAppHal_t appHal;
//void lcd_print_sx126x_opmode(/*const char**/);
//void sx126x_print_status(void);
//void lcd_print_sx126x_bw(uint8_t x, uint8_t y);
//void lcd_print_sx126x_sf(uint8_t x, uint8_t y);
//void sx126x_step_sf(bool up);
//void sx126x_step_bw(bool up);
extern volatile uint32_t cycleDur;
void setAppHal_sx126x(void);
void setAppHal_sx127x(void);

void delay_ticks(unsigned t);

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
