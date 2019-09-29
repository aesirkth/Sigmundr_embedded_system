/*
 * write_read.c
 *
 *  Created on: Sep 16, 2019
 *      Author: Nils
 */
#include "write_read.h"


//Fct to write a byte and test if the writing has been successful
uint32_t write8(uint8_t registerAdress, uint8_t command, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi)
{
	uint16_t Size = 2;
	uint8_t dataWrite[2] = {0,0};
	dataWrite[0] = registerAdress;
	dataWrite[1] = command;
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi, &dataWrite, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

//	TESTING IF WRITING WAS SUCCESSFUL
	uint32_t test = 0;
	uint8_t dataRead = 0;
	read8(registerAdress, &dataRead, GPIO_Port, Pin, hspi);
	if(dataRead == command){test=1;}
	return test;
}

//Fct to read a byte at the corresponding register adress
void read8(uint8_t registerAdress, uint8_t *data, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi)
{
	uint16_t Size = 2;
	uint8_t dataRead[2] = {0,0};
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);
	*data = dataRead[1];
}

//Fct to read 2*Number bytes and put them in N uint16_t form
uint16_t read16(uint8_t registerAdress, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi)
{
	uint16_t data = 0;
	uint16_t Size = 3;
	uint8_t dataRead[Size];
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

	data = dataRead[1]<<8 | dataRead[2];
	return data;
}
