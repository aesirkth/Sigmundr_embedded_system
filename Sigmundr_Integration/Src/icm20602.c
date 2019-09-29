/*
 * icm20602.c
 *
 *  Created on: Sep 5, 2019
 *      Author: Nils Lindberg Odhner
 */

//Fct to initialize the ICM20602 register, report errors in err variable
#include "icm20602.h"

unsigned int init_icm20602()
{
	unsigned int test = 0x00000011;
	uint8_t dataRead = 0;

	read8(ICM20602_WHO_AM_I, &dataRead, NSS_ICM_GPIO_Port, NSS_ICM_Pin);	//Read who I am register, and check if communication is working
	if(dataRead != ICM20602_ID){test &= 0x11111101;}

	write8(ICM20602_PWR_MGMT_1, ICM20602_RESET, NSS_ICM_GPIO_Port, NSS_ICM_Pin);
	HAL_Delay(10);
	if(write8(ICM20602_I2C_INTERFACE, ICM20602_DISABLE_I2C, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_PWR_MGMT_1, ICM20602_ENABLE_TEMP, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_PWR_MGMT_2, ICM20602_ENABLE_ACC_GYRO, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_SMPLRT_DIV, ICM20602_SAMPLE_RATE, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_CONFIG, ICM20602_LPFGYRO_176, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_ACCEL_CONFIG, ICM20602_SCALE16G_ACC, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_ACCEL_CONFIG2, ICM20602_LPFACC_99, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}
	if(write8(ICM20602_GYRO_CONFIG, ICM20602_SCALE500DPS_GYRO, NSS_ICM_GPIO_Port, NSS_ICM_Pin) != 1){test &= 0x11111110;}

	return test;
}

// Fct to read 2*Number bytes and put them in N uint16_t form
void read16N(uint8_t registerAdress, uint16_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef hspi)
{
	uint16_t Size = 1 + 2*Number;
	uint8_t dataRead[Size];
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

	uint32_t i = 0;
	for(i=0;i<Number;i++){
		*data++ = dataRead[1+2*i]<<8 | dataRead[2+2*i];
	}
}
