
/* D7   PI3  is radio NSS
 * D3   PB4  is radio busy
 * D13  PI1  is SCK
 * D12  PB14 is MISO
 * D11  PB15 is MOSI
 * D8   PI2  is ant-sw-power
 * D5   PI0  is dio1 interrupt in
 * A2   PF9  device detect
 * A3   PF8  xtal detect
 * A4   PF7  freq band detect
 */
#define DIO1_IRQHandler                 EXTI0_IRQHandler
#define DIO1_PIN                        GPIO_PIN_0
#define DIO1_PORT                       GPIOI
#define DIO1_IRQn                       EXTI0_IRQn
#define DIO1                            (HAL_GPIO_ReadPin(DIO1_PORT, DIO1_PIN) == GPIO_PIN_SET)


