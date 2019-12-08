/*
 * IMU.h
 *
 *  Created on: 7 oct. 2019
 *
 */

#ifndef IMU_H_
#define IMU_H_


#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "readWrite.h"

void Error_Handler(void);


uint32_t initIMU_fast(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
uint32_t initIMU_slow(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
void clearFIFO(GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
uint32_t readIMU(uint8_t *IMU_raw_data, uint16_t number, GPIO_TypeDef *NSS_GPIO_Port, uint16_t NSS_Pin, SPI_HandleTypeDef *hspiN);
void convIMU(uint8_t *IMU_raw_data, float *IMUConv, uint32_t cycle);

/* Register Position ---------------------------------------------------------*/
#define ICM_REG_PWR_MGMT_1 			0x6B
#define ICM_REG_PWR_MGMT_2 			0x6C
#define ICM_REG_I2C_INTERFACE 		0x70
#define ICM_REG_SMPLRT_DIV 			0x19
#define ICM_REG_CONFIG				0x1A
#define ICM_REG_GYRO_CONFIG			0x1B
#define ICM_REG_ACCEL_CONFIG		0x1C
#define ICM_REG_ACCEL_CONFIG2		0x1D
#define ICM_REG_WHO_AM_I			0x75
#define ICM_REG_FIFO_ENABLE			0x23
#define ICM_REG_USER_CONTROL		0x6A
#define ICM_REG_INT_DRDY			0x37
#define ICM_REG_FIFO_WATERMARK_H	0x60
#define ICM_REG_FIFO_WATERMARK_L	0x61
#define ICM_REG_FIFO_COUNT_H		0x72 //for reading : [15:8] Byte indicating number of bytes in the FIFO
#define ICM_REG_FIFO_COUNT_L		0x73 //for reading : [7:0] Byte indicating number of bytes in the FIFO

#define ICM_REG_DATA_REG			0x3b //goes from 0x3b to 0x48 (acc-temp-gyro)
#define ICM_REG_FIFO_REG 			0x74 //read FIFO

/* Commands -------------------------------------------------------------------*/
#define ICM_CMD_RESET				0x80 //ICM_PWR_MGMT_1 : reset all registers to default value
#define ICM_CMD_CLOCK_SOURCE		0x01 //ICM_PWR_MGMT_1 : clock source autoselect + enable temperature
#define ICM_CMD_DISABLE_I2C			0x40 //ICM_I2C_INTERFACE : disable I2C, activate SPI
#define ICM_CMD_ENABLE_ACC_GYRO 	0x00 //ICM_PWR_MGMT_2 : enable acc and gyro
#define ICM_CMD_SAMPLE_4			0xF9 //ICM_SMPLRT_DIV: 1000/(250)=4 Hz
#define ICM_CMD_SAMPLE_10			0x63 //ICM_SMPLRT_DIV: 1000/(100)=10 Hz
#define ICM_CMD_SAMPLE_125			0x07 //ICM_SMPLRT_DIV: 1000/(1+7)=125 Hz
#define ICM_CMD_SAMPLE_200			0x04 //ICM_SMPLRT_DIV: 1000/(1+4)=200 Hz
#define ICM_CMD_SAMPLE_500			0x01 //ICM_SMPLRT_DIV: 1000/(1+1)=500 Hz
#define ICM_CMD_SAMPLE_1000			0x00 //ICM_SMPLRT_DIV: 1000/(1+0)=1000Hz
#define ICM_CMD_LPFGYRO_176			0x01 //ICM_CONFIG : LPF gyro 176Hz (250,176,92,41Hz)
#define ICM_CMD_LPFGYRO_92			0x02 //ICM_CONFIG : LPF gyro 92Hz (250,176,92,41Hz)
#define ICM_CMD_LPFGYRO_41			0x03 //ICM_CONFIG : LPF gyro 41Hz (250,176,92,41Hz)
#define ICM_CMD_SCALE16G_ACC		0x18 //ICM_ACCEL_CONFIG : 16G full scale ACC
#define ICM_CMD_LPFACC_218			0x01 //ICM_ACCEL_CONFIG2 : LPG acc 218.1Hz (218,99,45Hz)
#define ICM_CMD_LPFACC_99			0x02 //ICM_ACCEL_CONFIG2 : LPG acc 99Hz (218,99,45Hz)
#define ICM_CMD_LPFACC_45			0x03 //ICM_ACCEL_CONFIG2 : LPG acc 45Hz (218,99,45Hz)
#define ICM_CMD_SCALE500DPS_GYRO	0x08 //ICM_GYRO_CONFIG : 500dps full scale gyro (250,500,1000,2000dps)
#define ICM_CMD_SCALE1000DPS_GYRO	0x10 //ICM_GYRO_CONFIG : 1000dps full scale gyro (250,500,1000,2000dps)
#define ICM_CMD_ACC_GYR_IN_FIFO		0x18 //ICM_FIFO_ENABLE : Put the Acc values, temp and Gyro in the Fifo
#define ICM_CMD_ACC_IN_FIFO			0x08 //ICM_FIFO_ENABLE : Put the Acc values and temp in the Fifo
#define ICM_CMD_SIG_COND_RST		0x41 //ICM_USER_CONTROL : reset all digital signal path including sensor register
#define ICM_CMD_RESET_FIFO			0x44 //ICM_USER_CONTROL : reset the FIFO module
#define ICM_CMD_ENABLE_FIFO_OP		0x41 //ICM_USER_CONTROL : enable FIFO operation and reset signal path
#define ICM_CMD_INT_GENERATION		0x30 //ICM_INT_DRDY : keep the interrupt signal high and put it low after read operation
#define ICM_CMD_FIFO_NBYTE_H_0		0x00 //ICM_FIFO_WATERMARK_H : Number of Bytes*256 in the FIFO before triggering an interrupt
#define ICM_CMD_FIFO_NBYTE_L_56		0x38 //ICM_FIFO_WATERMARK_L : Number of Bytes in the FIFO before triggering an interrupt
#define ICM_CMD_FIFO_NBYTE_L_140	0x8C //ICM_FIFO_WATERMARK_L : Number of Bytes in the FIFO before triggering an interrupt
#define ICM_CMD_ID					0x12 //IMU ID

#define ICM_CMD_FIFO_NBYTE_H		0x01
#define ICM_CMD_FIFO_NBYTE_L		0x18
#define	WATERMARK_IMU3				(ICM_CMD_FIFO_NBYTE_H<<8 | ICM_CMD_FIFO_NBYTE_L)
//#define WATERMARK_IMU2				(WATERMARK_IMU3 * 2 / 5)
#define WATERMARK_IMU2				4*14 //4 sample per period (50Hz), 14 bytes per sample


#endif /* IMU_H_ */
