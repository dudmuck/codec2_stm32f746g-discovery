#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <lr20xx_system_types.h>
#include <lr20xx_status.h>

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

extern uint8_t LR20xx_tx_buf[];    // lora fifo size
extern uint8_t LR20xx_rx_buf[];    // lora fifo size

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

extern lr20xx_system_chip_modes_t LR20xx_chipMode;
