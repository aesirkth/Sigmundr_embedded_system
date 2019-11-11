/*
 * MAG.h
 *
 *  Created on: 10 oct. 2019
 *      Author: noe
 */

#ifndef MAG_H_
#define MAG_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "readWrite.h"

void Error_Handler(void);

uint32_t initMAG(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin,  SPI_HandleTypeDef *hspiN);
void readMAG(uint8_t *MAG_raw_data, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
void convMAG(uint8_t *MAG_raw_data, float *MAGConv);


/* Register Position ---------------------------------------------------------*/
#define LIS_WHO_AM_I				0x8F //0x0F
#define LIS_CTRL_REG1				0x20
#define LIS_CTRL_REG2				0x21
#define LIS_CTRL_REG3				0x22
#define LIS_CTRL_REG4				0x23
#define LIS_DATA_REG				0xE8 //0x28 (reading + incrementing address)

/* Commands -------------------------------------------------------------------*/
#define LIS_ID						0x3d //LIS_WHO_AM_I : LIS ID
#define LIS_UHP						0x7E //LIS_CTRL_REG1 : Ultra high performance (axe X and Y), 155Hz
#define LIS_SCALE4G					0x00 //LIS_CTRL_REG2 : FS +/- 4Gauss (0.25-0.65 Gauss at earth's surface)
#define LIS_RESET					0x0C //LIS_CTRL_REG2 : Reboot and Soft reset of the device
#define LIS_ACTIVATE				0x00 //LIS_CTRL_REG3 : Activate continuous mode
#define LIS_ZUHP					0x0E //LIS_CTRL_REG4 : axe Z UHP

#endif /* MAG_H_ */
