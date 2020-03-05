
#define SPIx                             SPI2

#define SPIx_SCK_GPIO_CLK_ENABLE        __HAL_RCC_GPIOI_CLK_ENABLE
#define SPIx_MISO_GPIO_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE
#define SPIx_MOSI_GPIO_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE
#define SPIx_CLK_ENABLE                 __HAL_RCC_SPI2_CLK_ENABLE

#define SPIx_SCK_PIN                     GPIO_PIN_1
#define SPIx_SCK_GPIO_PORT               GPIOI
#define SPIx_SCK_AF                      GPIO_AF5_SPI2
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI2
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI2

#define NSS_SX126X_PIN                        GPIO_PIN_3
#define NSS_SX126X_PORT                       GPIOI
#define NSS_SX126X_GPIO_CLK_ENABLE            __HAL_RCC_GPIOI_CLK_ENABLE
#define ASSERT_SX126X_NSS                      HAL_GPIO_WritePin(NSS_SX126X_PORT, NSS_SX126X_PIN, GPIO_PIN_RESET)
#define UNASSERT_SX126X_NSS                    HAL_GPIO_WritePin(NSS_SX126X_PORT, NSS_SX126X_PIN, GPIO_PIN_SET)

#define NSS_SX127X_PIN                        GPIO_PIN_8
#define NSS_SX127X_PORT                       GPIOA
#define NSS_SX127X_GPIO_CLK_ENABLE            __HAL_RCC_GPIOA_CLK_ENABLE
#define ASSERT_SX127X_NSS                      HAL_GPIO_WritePin(NSS_SX127X_PORT, NSS_SX127X_PIN, GPIO_PIN_RESET)
#define UNASSERT_SX127X_NSS                    HAL_GPIO_WritePin(NSS_SX127X_PORT, NSS_SX127X_PIN, GPIO_PIN_SET)

uint8_t spi_transfer(uint8_t out_data);

