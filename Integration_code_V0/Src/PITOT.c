/*
 * PITOT.c
 *
 *  Created on: 10 oct. 2019
 *  This files contains the interface for pitot tube : ABPDRRT005PG2A5
 */

#include "PITOT.h"

//Reads the data from the pitot tube
uint16_t readPITOT(I2C_HandleTypeDef *hi2cN)
{
	uint8_t receive_buffer[2]={0};
	uint16_t output = 0;
	HAL_I2C_Master_Receive(hi2cN, PITOT_ADDRESS << 1, &receive_buffer, 2, 10); //need to left-shift the address
	output = receive_buffer[0]<<8 | receive_buffer[1];
	return output;
}


//Convert data of the PITOT tube in Pascal
float convPITOT(uint16_t PITOT_raw)
{
	uint16_t output_min = 1638, output_max = 14745; //10% to 90% calibration with 2^14 counts
	float pressure = 0.0, pressure_min = 0.0, pressure_max = 5.0 * 34474; //in Pa (1 PSI = 34474 Pa)
	pressure = (((float)(PITOT_raw - output_min))*(pressure_max - pressure_min) / (float)(output_max - output_min)) + pressure_min;
	return pressure;
}


//Convert data of the PITOT tube in m/s  p = 1/2 * rho * U^2. This is a rough estimation, can be improved.
float convSpeedPITOT(float pressure)
{
	float speed = 0.0, rho = 1.225;
	speed = sqrt(2*pressure/rho);	//sqrt is slow, rho=1.225 can surely be better tuned
	return speed;
}
