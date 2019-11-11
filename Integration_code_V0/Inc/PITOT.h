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

void readPITOT(uint8_t *PITOT_raw, I2C_HandleTypeDef *hi2cN);
void convSpeedPITOT(uint8_t *PITOT_raw, float *speed);

#define PITOT_ADDRESS		0x28	//I2C address of the sensor

#endif /* PITOT_H_ */
