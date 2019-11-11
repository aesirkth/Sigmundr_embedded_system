/*
 * PITOT.c
 *
 *  Created on: 10 oct. 2019
 *  This files contains the interface for pitot tube : ABPDRRT005PG2A5
 */

#include "PITOT.h"

//Reads the data from the pitot tube
void readPITOT(uint8_t *PITOT_raw, I2C_HandleTypeDef *hi2cN)
{
	HAL_I2C_Master_Receive(hi2cN, PITOT_ADDRESS << 1,(uint8_t*) PITOT_raw, 2, 2); //need to left-shift the address
}


//Convert data of the PITOT tube in m/s  p = 1/2 * rho * U^2. This is a rough estimation, can be improved.
void convSpeedPITOT(uint8_t *PITOT_raw, float *speed)
{
	uint16_t output;
	output = *PITOT_raw++ << 8 | *PITOT_raw;

	//Convert first the raw data to pressure
	uint16_t output_min = 1638, output_max = 14745; //10% to 90% calibration with 2^14 counts
	float pressure = 0.0, pressure_min = 0.0, pressure_max = 5.0 * 34474; //in Pa (1 PSI = 34474 Pa)
	pressure = (((float)(output - output_min))*(pressure_max - pressure_min) / (float)(output_max - output_min)) + pressure_min;

	//Convert the pressure to velocity
	float rho = 1.225;
	*speed = sqrt(2*pressure/rho);	//sqrt is slow, rho=1.225 can surely be better tuned
}
