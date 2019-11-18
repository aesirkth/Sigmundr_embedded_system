/*
 * readWrite.c
 *
 *  Created on: 10 oct. 2019
 *  This files contains the read_Write function for SPI protocol
 */

#include "readWrite.h"

//Fct to write (SPI) a byte and test if the writing has been successful
uint32_t write8(uint8_t registerAdress, uint8_t command, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 2;
	uint8_t dataWrite[2] = {0};
	dataWrite[0] = registerAdress;
	dataWrite[1] = command;
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspiN, (uint8_t*) &dataWrite, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

//	TESTING IF WRITING WAS SUCCESSFUL
	uint32_t test = 0;
	uint8_t dataRead = 0;
	read8(registerAdress, &dataRead, GPIO_Port, Pin, hspiN);
	if(dataRead == command){test=1;}
	return test;
}

//Fct to read a byte at the corresponding register address (SPI)
void read8(uint8_t registerAdress, uint8_t *data, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 2;
	uint8_t dataRead[2] = {0};
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, (uint8_t*) &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);
	*data = dataRead[1];
}

//Fct to read 2*Number bytes and put them in N uint16_t form (SPI)
void readN(uint8_t registerAdress, uint8_t *data, uint16_t number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	data[0] = registerAdress | 0x80;
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspiN, (uint8_t*) data, (uint8_t*) data, 1, 1);
	HAL_SPI_TransmitReceive(hspiN, (uint8_t*) data, (uint8_t*) data, number, 2);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);
}
