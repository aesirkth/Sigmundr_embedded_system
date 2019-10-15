/*
 * BMP.c
 *
 *  Created on: 7 oct. 2019
 *  This files contains the interface for the BMP : bmp280
 */

#include "BMP.h"


// This function is to read N 20bits data. (N should usually equal 2, pressure + temperature)
// Takes the register, the pointer to the data (signed int), the number of 20 bits data N, the CS Port and pin, as well as the SPI port.
void readBMP(int32_t *data, uint8_t Number, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 1 + 3*Number;
	uint8_t dataRead[Size];
	dataRead[0] = BMP280_DATA_REG | 0x80; 		//for a read command MSB should be 1
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, &dataRead, Size, 10);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

	unsigned int i = 0;
	for(i=0;i<Number;i++){
		*data++ = dataRead[1+3*i]<<12 | dataRead[2+3*i]<<4 | dataRead[3+3*i]; //signed int, 20 bit format !
	}
}


//Fct to read short uint16_t with LSB in the first position (for reading calibration parameters)
uint16_t read16BMP(uint8_t registerAdress, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t data = 0;
	uint16_t Size = 3;
	uint8_t dataRead[3] = {0,0,0};
	dataRead[0] = registerAdress | 0x80; 	//for a read command MSB should be 1
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, &dataRead, Size, 10);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

	data = dataRead[2]<<8 | dataRead[1]; 	//WARNING, unusual : MSB in the second position
	return data;
}


//This function is to read the calibration parameters from the BMP memory
void readParamBmp(param *bmp, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	bmp->T1 = (uint16_t) read16BMP(0x08, NSS_GPIO_Port, NSS_Pin, hspiN); 	//0x88, 0x08 since MSB added by the reading fct
	bmp->T2 = (int16_t) read16BMP(0x0A, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x8A
	bmp->T3 = (int16_t) read16BMP(0x0C, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x8C
	bmp->P1 = (uint16_t) read16BMP(0x0E, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x8E
	bmp->P2 = (int16_t) read16BMP(0x10, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x90
	bmp->P3 = (int16_t) read16BMP(0x12, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x92
	bmp->P4 = (int16_t) read16BMP(0x14, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x94
	bmp->P5 = (int16_t) read16BMP(0x16, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x96
	bmp->P6 = (int16_t) read16BMP(0x18, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x98
	bmp->P7 = (int16_t) read16BMP(0x1A, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x9A
	bmp->P8 = (int16_t) read16BMP(0x1C, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x9C
	bmp->P9 = (int16_t) read16BMP(0x1E, NSS_GPIO_Port, NSS_Pin, hspiN);	//0x9E
}


// Initialize the BMP sensors, take NSS pin and port and SPI port.
// Return 1 is successful, 0 otherwise
uint32_t initBMP(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint32_t verif = 0;
	uint8_t dataRead = 0;

	read8(BMP280_WHO_AM_I, &dataRead, NSS_GPIO_Port, NSS_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead == BMP280_ID){verif = 1;}

	write8(BMP280_RESET_REG, BMP280_RESET_CMD, NSS_GPIO_Port, NSS_Pin, hspiN);
	HAL_Delay(10);
	if(write8(BMP280_CTRL_MEAS, BMP280_OVERSAMPL_TEMP | BMP280_OVERSAMPL_PRESS | BMP280_MODE_NORMAL, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(BMP280_CONFIG, BMP280_TSB_05 | BMP280_IRR_16 | BMP280_EN_SPI3, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	return verif;
}

// Compensate the raw reading adc_T and adc_P from the baro
// T is given with a resolution of 0.01DegC, ie "5123" = 51.23°C
// divide p value to get the pressure in Pa. "24674867" = 24674867/256 = 96386.2 Pa
void bmpCompensate(param Bmp, int32_t bmpRaw[2], int32_t *bmpCompensated)
{
	int32_t t_fine;
	int32_t var1, var2, T;
	var1 = ((((bmpRaw[1]>>3) - ((int32_t)Bmp.T1<<1))) * ((int32_t)Bmp.T2)) >> 11;
	var2 = (((((bmpRaw[1]>>4) - ((int32_t)Bmp.T1)) * ((bmpRaw[1]>>4) - ((int32_t)Bmp.T1))) >> 12) * ((int32_t)Bmp.T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 +128) >> 8;
	*bmpCompensated++ = T;

	int64_t var11, var22, p;
	var11 = ((int64_t)t_fine) - 128000;
	var22 = var11 * var11 * (int64_t)Bmp.P6;
	var22 = var22 + ((var11*(int64_t)Bmp.P5)<<17);
	var22 = var22 + (((int64_t)Bmp.P4)<<35);
	var11 = ((var11 * var11 * (int64_t)Bmp.P3)>>8) + ((var11 * (int64_t)Bmp.P2)<<12);
	var11 = (((((int64_t)1)<<47)+var11))*((int64_t)Bmp.P1)>>33;
	if(var1==0){
		*bmpCompensated = (uint32_t) 0;
	}
	else{
		p = 1048576-bmpRaw[0];
		p = (((p<<31)-var22)*3125)/var11;
		var11 = (((int64_t)Bmp.P9) * (p>>13) * (p>>13)) >> 25;
		var22 = (((int64_t)Bmp.P8) * p) >> 19;
		p = ((p + var11 + var22) >> 8) + (((int64_t)Bmp.P7)<<4);
		*bmpCompensated = (uint32_t) p;
	}
}

//Performances, 72Mhz uC, 4.5Mhz SPI : 34us
//This function read the data from the BMP and compensate it, result is in bmpCompensated (pressure and temperature)
void readBMPCal(param Bmp, int32_t *bmpCompensated, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	int32_t data[2] = {0};
	int32_t bmpRaw[2] = {0};
	readBMP(&bmpRaw, 2, NSS_GPIO_Port, NSS_Pin, hspiN); //Read BMP data registers
	bmpCompensate(Bmp, bmpRaw, &data);
	*bmpCompensated++ = data[0];
	*bmpCompensated = data[1];
}

//Performances, 72Mhz uC, 4.5Mhz SPI : 12us
// Convert data into float to debug, check, processing, takes as input int32_t and float (booth 2 values)
// T is given with a resolution of 0.01DegC, ie "5123" = 51.23°C, divide p value to get the pressure in Pa. "24674867" = 24674867/256 = 96386.2 Pa
void convBMP(int32_t *bmpCompensated, float *bmpConv)
{
	*bmpConv++ = *bmpCompensated++ / 100.0;
	*bmpConv = *bmpCompensated / 256.0;
}
