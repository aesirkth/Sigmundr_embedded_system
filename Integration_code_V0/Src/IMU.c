/*
 * IMU.c
 *
 *  Created on: 7 oct. 2019
 *  This files contains the interface for the IMU : icm20602
 */

#include <unistd.h>
#include "IMU.h"

// Fct to initialize the ICM20602 registers, see IMU.h for the details of the registers
// It take the NSS (port and pin) of the sensors and its SPI port connection
// return 1 if initialization is successful (all registers written correctly), 0 otherwise
uint32_t initIMU_fast(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint32_t verif = 0;
	uint8_t dataRead = 0;

	read8(ICM_REG_WHO_AM_I, &dataRead, NSS_GPIO_Port, NSS_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead == ICM_CMD_ID){verif = 1;}

	write8(ICM_REG_PWR_MGMT_1, ICM_CMD_RESET, NSS_GPIO_Port, NSS_Pin, hspiN);
	HAL_Delay(10);
	write8(ICM_REG_USER_CONTROL			, ICM_CMD_RESET_FIFO		, NSS_GPIO_Port, NSS_Pin, hspiN); //autoclears
	write8(ICM_REG_USER_CONTROL			, ICM_CMD_ENABLE_FIFO_OP	, NSS_GPIO_Port, NSS_Pin, hspiN); //autoclears
	if(write8(ICM_REG_I2C_INTERFACE		, ICM_CMD_DISABLE_I2C		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_PWR_MGMT_1		, ICM_CMD_CLOCK_SOURCE		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_PWR_MGMT_2		, ICM_CMD_ENABLE_ACC_GYRO	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_ACCEL_CONFIG		, ICM_CMD_SCALE16G_ACC		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_GYRO_CONFIG		, ICM_CMD_SCALE1000DPS_GYRO	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_ACCEL_CONFIG2		, ICM_CMD_LPFACC_218		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_CONFIG			, ICM_CMD_LPFGYRO_176		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_SMPLRT_DIV		, ICM_CMD_SAMPLE_10			, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	if(write8(ICM_REG_FIFO_ENABLE		, ICM_CMD_ACC_GYR_IN_FIFO	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_INT_DRDY			, ICM_CMD_INT_GENERATION	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	//if(write8(ICM_REG_FIFO_WATERMARK_H	, ICM_CMD_FIFO_NBYTE_H_0	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	//if(write8(ICM_REG_FIFO_WATERMARK_L	, ICM_CMD_FIFO_NBYTE_L_140	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_FIFO_WATERMARK_H	, ICM_CMD_FIFO_NBYTE_H	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_FIFO_WATERMARK_L	, ICM_CMD_FIFO_NBYTE_L	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	return verif;
}

uint32_t initIMU_slow(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint32_t verif = 0;
	uint8_t dataRead = 0;

	read8(ICM_REG_WHO_AM_I, &dataRead, NSS_GPIO_Port, NSS_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead == ICM_CMD_ID){verif = 1;}

	write8(ICM_REG_PWR_MGMT_1, ICM_CMD_RESET, NSS_GPIO_Port, NSS_Pin, hspiN);
	HAL_Delay(10);
	write8(ICM_REG_USER_CONTROL			, ICM_CMD_RESET_FIFO		, NSS_GPIO_Port, NSS_Pin, hspiN); //autoclears
	write8(ICM_REG_USER_CONTROL			, ICM_CMD_ENABLE_FIFO_OP	, NSS_GPIO_Port, NSS_Pin, hspiN); //autoclears
	if(write8(ICM_REG_I2C_INTERFACE		, ICM_CMD_DISABLE_I2C		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_PWR_MGMT_1		, ICM_CMD_CLOCK_SOURCE		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_PWR_MGMT_2		, ICM_CMD_ENABLE_ACC_GYRO	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_ACCEL_CONFIG		, ICM_CMD_SCALE16G_ACC		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_GYRO_CONFIG		, ICM_CMD_SCALE1000DPS_GYRO	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_ACCEL_CONFIG2		, ICM_CMD_LPFACC_99			, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_CONFIG			, ICM_CMD_LPFGYRO_92		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}
	if(write8(ICM_REG_SMPLRT_DIV		, ICM_CMD_SAMPLE_200		, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	if(write8(ICM_REG_FIFO_ENABLE		, ICM_CMD_ACC_GYR_IN_FIFO	, NSS_GPIO_Port, NSS_Pin, hspiN) != 1){verif = 0;}

	return verif;
}


void clearFIFO(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t sizeFifo=0, readNumber=0;
	uint8_t data[3]={0};
	data[0] = ICM_REG_FIFO_COUNT_H | 0x80;
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspiN, (uint8_t*) &data, (uint8_t*) &data, 3, 1);
	sizeFifo = data[1]<<8 | data[2];

	readNumber = (sizeFifo/14)*14;
	uint8_t dummyTx[readNumber+1];
	uint8_t dummyRx[readNumber+1];
	dummyTx[0] = ICM_REG_FIFO_REG | 0x80;
	//HAL_SPI_TransmitReceive_DMA(hspiN, (uint8_t*) &dummyTx, (uint8_t*) &dummyRx, readNumber+1);
	HAL_SPI_TransmitReceive(hspiN, (uint8_t*) &dummyTx, (uint8_t*) &dummyRx, readNumber+1,100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

// Performances, 72Mhz uC, 4.5Mhz SPI :55us
// function to read number Bytes from the FIFO buffer. Return 1 if we read less than the number of bytes in the FIFO
// carefull sensor big endian, STM32 little endian
uint32_t readIMU(uint8_t *IMU_raw_data, uint16_t number, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t sizeFifo=0, readNumber=0;
	uint8_t data[3]={0};
	uint32_t iserror = 0;
	data[0] = ICM_REG_FIFO_COUNT_H | 0x80;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, (uint8_t*) &data, 3, 2);
	sizeFifo = data[1]<<8 | data[2];
	readNumber = (sizeFifo/14)*14;				//for reading a multiple of 14bytes (2bytes * 7 measurements)
	if(readNumber != number){iserror = 1;}
	IMU_raw_data[0] = ICM_REG_FIFO_REG | 0x80;
	HAL_SPI_TransmitReceive(hspiN, (uint8_t*) IMU_raw_data, (uint8_t*) IMU_raw_data, readNumber,5);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	//HAL_SPI_TransmitReceive_DMA(hspiN, (uint8_t*) IMU_raw_data, (uint8_t*) IMU_raw_data, readNumber);
	return iserror;
}

// Performances for 1 cycle, 72Mhz uC, 4.5Mhz SPI : 46us
// Convert data into float to debug, check, processing, takes as input float and int16_t (booth 7 values)
// If scale of the IMU is changed, this function has to be modified (see datasheet)
void convIMU(uint8_t *IMU_raw_data, float *IMUConv, uint32_t cycle)
{
	for(uint32_t i=0; i<cycle; i++){
		IMUConv[0+i*7] = (int16_t)(IMU_raw_data[0+i*14] << 8 | IMU_raw_data[1+i*14]) * (9.81 / 2048.0); 					//in m/s²
		IMUConv[1+i*7] = (int16_t)(IMU_raw_data[2+i*14] << 8 | IMU_raw_data[3+i*14]) * (9.81 / 2048.0);
		IMUConv[2+i*7] = (int16_t)(IMU_raw_data[4+i*14] << 8 | IMU_raw_data[5+i*14]) * (9.81 / 2048.0);
		IMUConv[3+i*7] = ((int16_t)(IMU_raw_data[6+i*14] << 8 | IMU_raw_data[7+i*14]) / 326.8) + 25.0;						//in °C
		IMUConv[4+i*7] = (int16_t)(IMU_raw_data[8+i*14] << 8 | IMU_raw_data[9+i*14]) / 32.8; 								//in dps
		IMUConv[5+i*7] = (int16_t)(IMU_raw_data[10+i*14] << 8 | IMU_raw_data[11+i*14]) / 32.8;
		IMUConv[6+i*7] = (int16_t)(IMU_raw_data[12+i*14] << 8 | IMU_raw_data[13+i*14]) / 32.8;
	}
}

