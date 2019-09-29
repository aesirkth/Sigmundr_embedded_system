/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
struct Param{
	  uint16_t T1, P1;
	  int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
};

typedef struct Param param;

uint32_t write8(uint8_t registerAdress, uint8_t command, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
void read8(uint8_t registerAdress, uint8_t *data, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
uint16_t read16(uint8_t registerAdress, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
void read16N(uint8_t registerAdress, uint16_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
void readBMP(uint8_t registerAdress, int32_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
int16_t readPitot();
unsigned int init_icm20602(GPIO_TypeDef *NSS_ICM_GPIO_Port, uint16_t NSS_ICM_Pin, SPI_HandleTypeDef *hspiN);
unsigned int init_bmp280(GPIO_TypeDef *NSS_BMP_GPIO_Port, uint16_t NSS_BMP_Pin, SPI_HandleTypeDef *hspiN);
unsigned int init_lis(GPIO_TypeDef *GPIO_Port, uint16_t Pin);
void readParamBmp(param *bmp, GPIO_TypeDef *NSS_BMP_GPIO_Port, uint16_t NSS_BMP_Pin, SPI_HandleTypeDef *hspiN);
void bmpCompensate(param Bmp, int32_t bmpRaw[2], int32_t *bmpCompensated);
unsigned int mainInitialization(param *Bmp1, param *Bmp2);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NSS_BMP1_Pin GPIO_PIN_13
#define NSS_BMP1_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOC
#define NSS_BMP2_Pin GPIO_PIN_9
#define NSS_BMP2_GPIO_Port GPIOA
#define NSS_LIS2_Pin GPIO_PIN_10
#define NSS_LIS2_GPIO_Port GPIOA
#define NSS_ICM2_Pin GPIO_PIN_11
#define NSS_ICM2_GPIO_Port GPIOA
#define NSS_NEO2_Pin GPIO_PIN_12
#define NSS_NEO2_GPIO_Port GPIOA
#define NSS_ICM1_Pin GPIO_PIN_15
#define NSS_ICM1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
//REGISTER POSITION
	//IMU
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
	//BMP
#define BMP280_WHO_AM_I				0xD0
#define BMP280_RESET_REG			0xE0
#define BMP280_CTRL_MEAS			0x74 //0xF4 BMP280_OVERSAMPL_TEMP | BMP280_OVERSAMPL_PRESS | BMP280_MODE_NORMAL
#define BMP280_CONFIG				0x75 //0xF5 BMP280_TSB_05 | BMP280_IRR_16 | BMP280_EN_SPI3
#define BMP280_DATA_REG				0xF7 //goes from 0xF7 to 0xFC (press-temp) 3 bytes for each
	//LIS : Write operation MSb=0 (read LSb=1), second MSb 1 for incrementing address (0 for non incrementing)
#define LIS_WHO_AM_I				0x8F //0x0F
#define LIS_CTRL_REG1				0x20
#define LIS_CTRL_REG2				0x21
#define LIS_CTRL_REG3				0x22
#define LIS_CTRL_REG4				0x23
#define LIS_DATA_REG				0xE8 //0x28 (reading + incrementing address)
	//PITOT PRESSURE
#define I2C_PITOT_ADDRESS			0x28 //address of the pressure sensor on the I2C

//COMMAND
	//IMU
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
	//BMP
#define BMP280_ID					0x58 //BMP280_WHO_AM_I : BMP ID
#define BMP280_RESET_CMD			0xB6 //BMP280_RESET_REG : BMP reset command
#define BMP280_OVERSAMPL_TEMP		0x20 //BMP280_CTRL_MEAS : temperature oversampling*1
#define BMP280_OVERSAMPL_PRESS		0x10 //BMP280_CTRL_MEAS : pressure oversampling*8 (high resolution)
#define BMP280_MODE_NORMAL			0x03 //BMP280_CTRL_MEAS : normal mode
#define BMP280_MODE_SLEEP			0x00 //BMP280_CTRL_MEAS : sleep mode
#define BMP280_TSB_05				0x00 //BMP280_CONFIG : standby time = 0.5ms (with high resolution = 50Hz)
#define BMP280_IRR_16				0x14 //BMP280_CONFIG : IRR filter 16
#define BMP280_EN_SPI3				0x00 //BMP280_CONFIG : do not select 3 wire SPI
	//LIS
#define LIS_ID						0x3d //LIS_WHO_AM_I : LIS ID
#define LIS_UHP						0x10 //LIS_CTRL_REG1 : Ultra high performance (axe X and Y), 155Hz
#define LIS_SCALE4G					0x00 //LIS_CTRL_REG2 : FS +/- 4Gauss
#define LIS_RESET					0x04 //LIS_CTRL_REG2 : Soft reset of the device
#define LIS_ACTIVATE				0x00 //LIS_CTRL_REG3 : Activate continuous mode
#define LIS_ZUHP					0x0E //LIS_CTRL_REG4 : axe Z UHP

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
