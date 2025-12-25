/* A0   PA0  is radio reset (NRST)
 * A1   PF10 is radio DIO8 interrupt in
 * D3   PB4  is radio busy
 */

#define DIO8_IRQHandler                 EXTI15_10_IRQHandler
#define DIO8_PIN                        GPIO_PIN_10
#define DIO8_PORT                       GPIOF
#define DIO8_GPIO_CLK_ENABLE            __HAL_RCC_GPIOF_CLK_ENABLE
#define DIO8_IRQn                       EXTI15_10_IRQn
#define DIO8                            (HAL_GPIO_ReadPin(DIO8_PORT, DIO8_PIN) == GPIO_PIN_SET)

#define BUSY_PIN                        GPIO_PIN_4
#define BUSY_PORT                       GPIOB
#define BUSY_GPIO_CLK_ENABLE            __HAL_RCC_GPIOB_CLK_ENABLE
#define BUSY                            (HAL_GPIO_ReadPin(BUSY_PORT, BUSY_PIN) == GPIO_PIN_SET)

#define NRST_PIN                        GPIO_PIN_0
#define NRST_PORT                       GPIOA
#define NRST_GPIO_CLK_ENABLE            __HAL_RCC_GPIOA_CLK_ENABLE
