#include <stdio.h>
#include "main.h"
#include "radio.h"
#include "lr20xx.h"
#include "fhss.h"
#include "stm32746g_discovery_lcd.h"

static void lcd_print_lr20xx_opmode(bool tx_wait)
{
	char str[16];
	const char* modeStr;
	uint8_t chipMode = get_chip_mode_lr20xx();

	switch (chipMode) {
		case 0: // SLEEP
		case 1: // STBY_RC
		case 2: // STBY_XOSC
		case 3: // FS
			modeStr = "       ";  /* 7 spaces to clear "chNN TX" */
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			break;
		case 4: // RX
			BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			modeStr = "RX     ";  /* Pad to 7 chars */
			break;
		case 5: // TX
			BSP_LCD_SetBackColor(LCD_COLOR_RED);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			sprintf(str, "ch%02u TX", fhss_cfg.current_channel);
			modeStr = str;
			break;
		default:
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			sprintf(str, "%d", chipMode);
			modeStr = str;
			break;
	}

	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-50, (uint8_t*)modeStr, RIGHT_MODE);
}

static void lr20xx_print_status()
{
	lorahal.printOpMode();
	printf("%uKHz SF%u ", get_bw_khz_lr20xx(), get_sf_lr20xx());
	printf("%luHz\r\n", (unsigned long)get_freq_hz_lr20xx());
}

static void lcd_print_lr20xx_bw(uint8_t x, uint8_t y)
{
	char str[16];
	sprintf(str, "%uKHz   ", get_bw_khz_lr20xx());
	BSP_LCD_DisplayStringAt(x, y, (uint8_t *)str, LEFT_MODE);
}

static void lcd_print_lr20xx_sf(uint8_t x, uint8_t y)
{
	char str[16];
	sprintf(str, "SF%u  ", get_sf_lr20xx());
	BSP_LCD_DisplayStringAt(x, y, (uint8_t *)str, LEFT_MODE);
}

void lr20xx_step_sf(bool up)
{
	step_sf_lr20xx(up);
}

void lr20xx_step_bw(bool up)
{
	step_bw_lr20xx(up);
}

void setAppHal_lr20xx()
{
    appHal.lcd_printOpMode = lcd_print_lr20xx_opmode;
    appHal.radioPrintStatus = lr20xx_print_status;
    appHal.lcd_print_bw = lcd_print_lr20xx_bw;
    appHal.lcd_print_sf = lcd_print_lr20xx_sf;
    appHal.step_bw = lr20xx_step_bw;
    appHal.step_sf = lr20xx_step_sf;
}
