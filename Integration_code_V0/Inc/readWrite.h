/*
 * readWrite.h
 *
 *  Created on: 10 oct. 2019
 *      Author: noe
 */

#ifndef READWRITE_H_
#define READWRITE_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

void Error_Handler(void);

uint32_t write8(uint8_t registerAdress, uint8_t command, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
void read8(uint8_t registerAdress, uint8_t *data, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);
void read16N(uint8_t registerAdress, uint16_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN);

#endif /* READWRITE_H_ */
