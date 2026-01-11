#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <lr20xx_system_types.h>
#include <lr20xx_status.h>
#include <lr20xx_radio_fifo_types.h>

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*!
 * @brief Helper macro that prints an error message if a command does not return LR20XX_STATUS_OK
 *
 * @param[in] rc  Return code from lr20xx_driver function
 */
#define ASSERT_LR20XX_RC( rc )                                                                   \
    {                                                                                            \
        const lr20xx_status_t status = rc;                                                       \
        if( status != LR20XX_STATUS_OK )                                                         \
        {                                                                                        \
            if( status == LR20XX_STATUS_ERROR )                                                  \
            {                                                                                    \
                printf( "LR20XX ERROR: In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__, \
                        xstr( LR20XX_STATUS_ERROR ) );                                           \
            }                                                                                    \
        }                                                                                        \
    }

void init_lr20xx(void);

/* Buffer size for TX and RX */
#define LR20XX_BUF_SIZE  1536  /* ~9 frames at 160 bytes for 32K mode */
extern uint8_t LR20xx_tx_buf[LR20XX_BUF_SIZE];
extern uint8_t LR20xx_rx_buf[LR20XX_BUF_SIZE];

bool LR20xx_service(void);
void LR20xx_setStandby(lr20xx_system_standby_mode_t stby_mode);
void init_lr20xx(void);

extern void (*LR20xx_dio8_topHalf)(void);
extern void (*LR20xx_timeout)(bool tx);
extern void (*LR20xx_txDone)(void);
extern void (*LR20xx_rxDone)(uint8_t size, float rssi, float snr);
extern void (*LR20xx_chipModeChange)(void);
extern void (*LR20xx_cadDone)(bool detected);
extern void (*LR20xx_preambleDetected)(void);
extern void (*LR20xx_fifoTx)(lr20xx_radio_fifo_flag_t tx_fifo_flags);
extern void (*LR20xx_fifoRx)(lr20xx_radio_fifo_flag_t rx_fifo_flags);

extern lr20xx_system_chip_modes_t LR20xx_chipMode;
