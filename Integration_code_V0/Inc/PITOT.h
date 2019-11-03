/*
 * PITOT.h
 *
 *  Created on: 10 oct. 2019
 *      Author: noe
 */

#ifndef PITOT_H_
#define PITOT_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

void Error_Handler(void);

uint16_t readPITOT(I2C_HandleTypeDef *hi2cN);
float convSpeedPITOT(uint16_t PITOT_raw);

#define PITOT_ADDRESS		0x28	//I2C address of the sensor

#endif /* PITOT_H_ */
