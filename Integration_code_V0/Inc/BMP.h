/*
 * BMP.h
 *
 *  Created on: 7 oct. 2019
 *      Author: noe
 */

#ifndef BMP_H_
#define BMP_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "readWrite.h"

void Error_Handler(void);


struct Param{
	  uint16_t T1, P1;
	  int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
};

typedef struct Param param;

void readBMP(int32_t *data, uint8_t Number, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
uint16_t read16BMP(uint8_t registerAdress, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
void readParamBmp(param *bmp, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
uint32_t initBMP(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
void bmpCompensate(int32_t bmpRaw[2], int32_t *bmpCompensated, param Bmp);
void readBMPCal(int32_t *bmpCompensated, param Bmp, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
void convBMP(int32_t *bmpCompensated, float *bmpConv);

/* Register Position ---------------------------------------------------------*/
#define BMP280_WHO_AM_I				0xD0
#define BMP280_RESET_REG			0xE0
#define BMP280_CTRL_MEAS			0x74 //0xF4 BMP280_OVERSAMPL_TEMP | BMP280_OVERSAMPL_PRESS | BMP280_MODE_NORMAL
#define BMP280_CONFIG				0x75 //0xF5 BMP280_TSB_05 | BMP280_IRR_16 | BMP280_EN_SPI3
#define BMP280_DATA_REG				0xF7 //goes from 0xF7 to 0xFC (press-temp) 3 bytes for each


/* Commands -------------------------------------------------------------------*/
#define BMP280_ID					0x58 //BMP280_WHO_AM_I : BMP ID
#define BMP280_RESET_CMD			0xB6 //BMP280_RESET_REG : BMP reset command
#define BMP280_OVERSAMPL_TEMP		0x20 //BMP280_CTRL_MEAS : temperature oversampling*1
#define BMP280_OVERSAMPL_PRESS		0x10 //BMP280_CTRL_MEAS : pressure oversampling*8 (high resolution)
#define BMP280_MODE_NORMAL			0x03 //BMP280_CTRL_MEAS : normal mode
#define BMP280_MODE_SLEEP			0x00 //BMP280_CTRL_MEAS : sleep mode
#define BMP280_TSB_05				0x00 //BMP280_CONFIG : standby time = 0.5ms (with high resolution = 50Hz)
#define BMP280_IRR_8				0x0C //BMP280_CONFIG : IRR filter 8
#define BMP280_IRR_16				0x14 //BMP280_CONFIG : IRR filter 16
#define BMP280_EN_SPI3				0x00 //BMP280_CONFIG : do not select 3 wire SPI

#endif /* BMP_H_ */
