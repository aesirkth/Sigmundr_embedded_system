/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WRITE_READ_H
#define __WRITE_READ_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t write8(uint8_t registerAdress, uint8_t command, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi);
void read8(uint8_t registerAdress, uint8_t *data, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi);
uint16_t read16(uint8_t registerAdress, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi);
/* USER CODE END EFP */


#endif /* WRITE_READ_H_ */
