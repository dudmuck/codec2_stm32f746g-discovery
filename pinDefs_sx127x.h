/*  D10 PA8 is radio NSS
 *  D13 PI1 is SCK
 *  D12 PB14 is MISO
 *  D11 PB15 is MOSI
 *  D2  PG6  is dio0 interrupt in
 *  A4  PF7  is RxTx
 *  A0  PA0  is nRST
 */
#define RFSW_GPIO_CLK_ENABLE            __HAL_RCC_GPIOF_CLK_ENABLE
#define RFSW_PIN                        GPIO_PIN_7
#define RFSW_PORT                       GPIOF
#define CLEAR_RFSW                      HAL_GPIO_WritePin(RFSW_PORT, RFSW_PIN, GPIO_PIN_RESET)
#define SET_RFSW                        HAL_GPIO_WritePin(RFSW_PORT, RFSW_PIN, GPIO_PIN_SET)

#define DIO0_GPIO_CLK_ENABLE            __HAL_RCC_GPIOF_CLK_ENABLE
#define DIO0_IRQHandler                 EXTI9_5_IRQHandler
#define DIO0_PIN                        GPIO_PIN_6
#define DIO0_PORT                       GPIOG
#define DIO0_IRQn                       EXTI9_5_IRQn
#define DIO0                            (HAL_GPIO_ReadPin(DIO0_PORT, DIO0_PIN) == GPIO_PIN_SET)
#define IS_DIO0_CLR                     (HAL_GPIO_ReadPin(DIO0_PORT, DIO0_PIN) == GPIO_PIN_RESET)

