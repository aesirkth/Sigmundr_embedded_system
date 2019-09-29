/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICM20602_H
#define __ICM20602_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdint.h"
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
unsigned int init_icm20602(void);
void read16N(uint8_t registerAdress, uint16_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define NSS_ICM_Pin GPIO_PIN_15
#define NSS_ICM_GPIO_Port GPIOA

/* Register Position ---------------------------------------------------------*/
#define ICM20602_PWR_MGMT_1 		0x6b
#define ICM20602_PWR_MGMT_2 		0x6c
#define ICM20602_SMPLRT_DIV 		0x19
#define ICM20602_CONFIG				0x1a
#define ICM20602_GYRO_CONFIG		0x1b
#define ICM20602_ACCEL_CONFIG		0x1c
#define ICM20602_ACCEL_CONFIG2		0x1d
#define ICM20602_I2C_INTERFACE 		0x70
#define ICM20602_WHO_AM_I			0x75

#define ICM20602_DATA_REG			0x3b //goes from 0x3b to 0x48 (acc-temp-gyro)

/* Commands -------------------------------------------------------------------*/
#define ICM20602_RESET				0x80 //ICM20602_PWR_MGMT_1 : reset all registers to default value
#define ICM20602_ENABLE_TEMP		0x01 //ICM20602_PWR_MGMT_1 : clock source autoselect + enable temperature
#define ICM20602_DISABLE_I2C		0x40 //ICM20602_I2C_INTERFACE : disable I2C, activate SPI
#define ICM20602_ENABLE_ACC_GYRO 	0x00 //ICM20602_PWR_MGMT_2 : enable acc and gyro
#define ICM20602_SAMPLE_RATE		0x07 //ICM20602_SMPLRT_DIV: 1000/(1+sample_rate)=125
#define ICM20602_LPFGYRO_176		0x01 //ICM20602_CONFIG : LPF gyro 176Hz (250,176,92,41Hz)
#define ICM20602_SCALE16G_ACC		0x18 //ICM20602_ACCEL_CONFIG : 16G full scale ACC
#define ICM20602_LPFACC_99			0x02 //ICM20602_ACCEL_CONFIG2 : LPG acc 99Hz (218,99,45Hz)
#define ICM20602_SCALE500DPS_GYRO	0x08 //ICM20602_GYRO_CONFIG : 500dps full scale gyro (250,500,1000,2000dps)
#define ICM20602_ID					0x12 //IMU ID

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
