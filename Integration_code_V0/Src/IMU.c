/*
 * IMU.c
 *
 *  Created on: 7 oct. 2019
 *  This files contains the interface for the IMU : icm20602
 */

#include "IMU.h"

//Fct to initialize the ICM20602 registers, see IMU.h for the details of the registers
//It take the NSS (port and pin) of the sensors and its SPI port connection
//return 1 if initialization is successful (all registers written correctly), 0 otherwise
uint32_t initIMU(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint32_t verif = 0;
	uint8_t dataRead = 0;

	read8(ICM20602_WHO_AM_I, &dataRead, NSS_GPIO_Port, NSS_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead == ICM20602_ID){verif = 1;}

	write8(ICM20602_PWR_MGMT_1, ICM20602_RESET, NSS_GPIO_Port, NSS_Pin, hspiN);
	HAL_Delay(10);
	if(write8(ICM20602_I2C_INTERFACE, ICM20602_DISABLE_I2C, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_PWR_MGMT_1, ICM20602_ENABLE_TEMP, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_PWR_MGMT_2, ICM20602_ENABLE_ACC_GYRO, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_SMPLRT_DIV, ICM20602_SAMPLE_RATE, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_CONFIG, ICM20602_LPFGYRO_176, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_ACCEL_CONFIG, ICM20602_SCALE16G_ACC, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_ACCEL_CONFIG2, ICM20602_LPFACC_99, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM20602_GYRO_CONFIG, ICM20602_SCALE500DPS_GYRO, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	return verif;
}

//Performances, 72Mhz uC, 4.5Mhz SPI :55us
//function to read (and cast) the data from the IMU, data is put in *IMU_raw_data which must contain 7 data
void readIMU(int16_t *IMU_raw_data, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t data[7] = {0};
	read16N(ICM20602_DATA_REG, &data, 7, NSS_GPIO_Port, NSS_Pin, hspiN);
	*IMU_raw_data++ = (int16_t) data[0];
	*IMU_raw_data++ = (int16_t) data[1];
	*IMU_raw_data++ = (int16_t) data[2];
	*IMU_raw_data++ = (int16_t) data[3];
	*IMU_raw_data++ = (int16_t) data[4];
	*IMU_raw_data++ = (int16_t) data[5];
	*IMU_raw_data = (int16_t) data[6];
}

//Performances, 72Mhz uC, 4.5Mhz SPI : 46us
// Convert data into float to debug, check, processing, takes as input float and int16_t (booth 7 values)
//If scale of the IMU is changed, this function has to be modified (see datasheet)
void convIMU(int16_t *IMU_raw_data, float *IMUConv)
{
	*IMUConv++ = *IMU_raw_data++ / 2048.0; 									//in g
	*IMUConv++ = *IMU_raw_data++ / 2048.0;
	*IMUConv++ = *IMU_raw_data++ / 2048.0;
	*IMUConv++ = (*IMU_raw_data++ / 326.8) + 25.0;							//in °C
	*IMUConv++ = *IMU_raw_data++ / 65.5; 									//in dps
	*IMUConv++ = *IMU_raw_data++ / 65.5;
	*IMUConv = *IMU_raw_data / 65.5;
}

