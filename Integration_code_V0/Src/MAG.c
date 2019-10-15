/*
 * MAG.c
 *
 *  Created on: 10 oct. 2019
 *  This files contains the interface for the magnetometer : lis3mdl
 */

#include "MAG.h"


//Fct to initialize the MAG registers, see MAG.h for the details of the registers
//It take the NSS (port and pin) of the sensors and its SPI port connection
//return 1 if initialization is successful (all registers written correctly), 0 otherwise
uint32_t initMAG(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin,  SPI_HandleTypeDef *hspiN)
{
	uint32_t verif = 0;
	uint8_t dataRead = 0;

	read8(LIS_WHO_AM_I, &dataRead, NSS_GPIO_Port, NSS_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead == LIS_ID){verif = 1;}

	write8(LIS_CTRL_REG2, LIS_RESET, NSS_GPIO_Port, NSS_Pin, hspiN);
	HAL_Delay(10);
	if(write8(LIS_CTRL_REG1, LIS_UHP, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(LIS_CTRL_REG4, LIS_ZUHP, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(LIS_CTRL_REG3, LIS_ACTIVATE, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	return verif;
}

//Performances, 72Mhz uC, 4.5Mhz SPI : 29us
//Read MAG data registers
void readMAG(int16_t *MAG_raw_data, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t data[3] = {0};
	read16N(LIS_DATA_REG, &data, 3, NSS_GPIO_Port, NSS_Pin, hspiN);
	*MAG_raw_data++ = (int16_t) data[0];
	*MAG_raw_data++ = (int16_t) data[1];
	*MAG_raw_data = (int16_t) data[2];
}

//Performances, 72Mhz uC, 4.5Mhz SPI : 29us
//Convert data in Gauss (float)
void convMAG(int16_t *MAG_raw_data, float *MAGConv)
{
	*MAGConv++ = *MAG_raw_data++ / 6842.0; 			//in Gauss
	*MAGConv++ = *MAG_raw_data++ / 6842.0;
	*MAGConv = *MAG_raw_data / 6842.0;
}
