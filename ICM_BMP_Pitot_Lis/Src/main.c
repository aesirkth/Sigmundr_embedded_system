/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Fct to write a byte and test if the writing has been successful
uint32_t write8(uint8_t registerAdress, uint8_t command, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 2;
	uint8_t dataWrite[2] = {0,0};
	dataWrite[0] = registerAdress;
	dataWrite[1] = command;
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspiN, &dataWrite, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

//	TESTING IF WRITING WAS SUCCESSFUL
	uint32_t test = 0;
	uint8_t dataRead = 0;
	read8(registerAdress, &dataRead, GPIO_Port, Pin, hspiN);
	if(dataRead == command){test=1;}
	return test;
}

//Fct to read a byte at the corresponding register adress
void read8(uint8_t registerAdress, uint8_t *data, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 2;
	uint8_t dataRead[2] = {0,0};
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET); //HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);NSS_ICM_GPIO_Port, NSS_ICM_Pin, GPIO_PIN_RESET
	HAL_SPI_Receive(hspiN, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET); //HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);NSS_ICM_GPIO_Port, NSS_ICM_Pin, GPIO_PIN_SET
	*data = dataRead[1];
}

//Fct to read short uint16_t with LSB in the first position
uint16_t read16(uint8_t registerAdress, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t data = 0;
	uint16_t Size = 3;
	uint8_t dataRead[3] = {0,0,0};
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

	data = dataRead[2]<<8 | dataRead[1]; //WARNING, unusual : MSB in the second position
	return data;
}

//Fct to read 2*Number bytes and put them in N uint16_t form
void read16N(uint8_t registerAdress, uint16_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 1 + 2*Number;
	uint8_t dataRead[Size];
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

	uint32_t i = 0;
	for(i=0;i<Number;i++){
		*data++ = dataRead[1+2*i]<<8 | dataRead[2+2*i];
	}
}

void readBMP(uint8_t registerAdress, int32_t *data, uint8_t Number, GPIO_TypeDef *GPIO_Port, uint16_t Pin, SPI_HandleTypeDef *hspiN)
{
	uint16_t Size = 1 + 3*Number;
	uint8_t dataRead[Size];
	dataRead[0] = registerAdress | 0x80; //for a read command MSB should be 1
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspiN, &dataRead, Size, 10);
	HAL_GPIO_WritePin(GPIO_Port, Pin, GPIO_PIN_SET);

	unsigned int i = 0;
	for(i=0;i<Number;i++){
		*data++ = dataRead[1+3*i]<<12 | dataRead[2+3*i]<<4 | dataRead[3+3*i]; //signed int, 20 bit format !
	}
}

int16_t readPitot()
{
	uint8_t receive_buffer[2];
	uint16_t output, output_min = 1638, output_max = 14745; //10% to 90% calibration with 2^14 counts
	int16_t pressure, pressure_min = -5, pressure_max = 5; //in PSI
	HAL_I2C_Master_Receive(&hi2c1, I2C_PITOT_ADDRESS, &receive_buffer, 2, 100); //maybe do need a write fct before
	output = receive_buffer[0]<<8 | receive_buffer[1];
	pressure = ((output - output_min)*(pressure_max - pressure_min) / (output_max - output_min)) + pressure_min;
	return pressure;
}

//Fct to initialize the ICM20602 register, report errors in err variable
unsigned int init_icm20602(GPIO_TypeDef *NSS_ICM_GPIO_Port, uint16_t NSS_ICM_Pin, SPI_HandleTypeDef *hspiN)
{
	unsigned int test = 0x00000011;
	uint8_t dataRead = 0;

	read8(ICM20602_WHO_AM_I, &dataRead, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead != ICM20602_ID){test &= 0x11111101;}

	write8(ICM20602_PWR_MGMT_1, ICM20602_RESET, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN);
	HAL_Delay(100);
	if(write8(ICM20602_I2C_INTERFACE, ICM20602_DISABLE_I2C, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_PWR_MGMT_1, ICM20602_ENABLE_TEMP, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_PWR_MGMT_2, ICM20602_ENABLE_ACC_GYRO, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_SMPLRT_DIV, ICM20602_SAMPLE_RATE, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_CONFIG, ICM20602_LPFGYRO_176, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_ACCEL_CONFIG, ICM20602_SCALE16G_ACC, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_ACCEL_CONFIG2, ICM20602_LPFACC_99, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(ICM20602_GYRO_CONFIG, ICM20602_SCALE500DPS_GYRO, NSS_ICM_GPIO_Port, NSS_ICM_Pin, hspiN) != 1){test &= 0x11111110;}

	return test;
}

unsigned int init_bmp280(GPIO_TypeDef *NSS_BMP_GPIO_Port, uint16_t NSS_BMP_Pin, SPI_HandleTypeDef *hspiN)
{
	unsigned int test = 0x00000011;
	uint8_t dataRead = 0;

	read8(BMP280_WHO_AM_I, &dataRead, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//Read who I am register, and check if communication is working
	if(dataRead != BMP280_ID){test &= 0x11111101;}

	write8(BMP280_RESET_REG, BMP280_RESET_CMD, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);
	HAL_Delay(100);
	if(write8(BMP280_CTRL_MEAS, BMP280_OVERSAMPL_TEMP | BMP280_OVERSAMPL_PRESS | BMP280_MODE_NORMAL, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN) != 1){test &= 0x11111110;}
	if(write8(BMP280_CONFIG, BMP280_TSB_05 | BMP280_IRR_16 | BMP280_EN_SPI3, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN) != 1){test &= 0x11111110;}

	return test;
}

unsigned int init_lis(GPIO_TypeDef *GPIO_Port, uint16_t Pin)
{
	unsigned int test = 0x00000011;
	uint8_t dataRead = 0;

	read8(LIS_WHO_AM_I, &dataRead, NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, &hspi2);	//Read who I am register, and check if communication is working
	if(dataRead != LIS_ID){test &= 0x11111101;}

	write8(LIS_CTRL_REG2, LIS_RESET, NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, &hspi2);
	HAL_Delay(100);
	if(write8(LIS_CTRL_REG1, LIS_UHP, NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, &hspi2) != 1){test &= 0x11111110;}
	if(write8(LIS_CTRL_REG4, LIS_ZUHP, NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, &hspi2) != 1){test &= 0x11111110;}
	if(write8(LIS_CTRL_REG3, LIS_ACTIVATE, NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, &hspi2) != 1){test &= 0x11111110;}

	return test;
}

void readParamBmp(param *bmp, GPIO_TypeDef *NSS_BMP_GPIO_Port, uint16_t NSS_BMP_Pin, SPI_HandleTypeDef *hspiN)
{
	bmp->T1 = (uint16_t) read16(0x08, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN); 	//0x88, 0x80 will be added from the reading instruction
	bmp->T2 = (int16_t) read16(0x0A, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x8A
	bmp->T3 = (int16_t) read16(0x0C, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x8C
	bmp->P1 = (uint16_t) read16(0x0E, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x8E
	bmp->P2 = (int16_t) read16(0x10, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x90
	bmp->P3 = (int16_t) read16(0x12, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x92
	bmp->P4 = (int16_t) read16(0x14, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x94
	bmp->P5 = (int16_t) read16(0x16, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x96
	bmp->P6 = (int16_t) read16(0x18, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x98
	bmp->P7 = (int16_t) read16(0x1A, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x9A
	bmp->P8 = (int16_t) read16(0x1C, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x9C
	bmp->P9 = (int16_t) read16(0x1E, NSS_BMP_GPIO_Port, NSS_BMP_Pin, hspiN);	//0x9E
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

//Main initialization function. Sensors denoted 1 are on the OBC board and use the SPI3 bus, Sensors denoted 2 are on the SB and use SPI2 bus.
unsigned int mainInitialization(param *Bmp1, param *Bmp2)
{
	unsigned int err=0;
	if(init_icm20602(NSS_ICM1_GPIO_Port, NSS_ICM1_Pin, &hspi3) != 0x00000011){//do the init and if failure report it to the error variable + put on led off
		err |= 0x00000001;
	}
	if(init_icm20602(NSS_ICM2_GPIO_Port, NSS_ICM2_Pin, &hspi2) != 0x00000011){//do the init and if failure report it to the error variable + put on led off
		err |= 0x00000010;
	}
	if(init_bmp280(NSS_BMP1_GPIO_Port, NSS_BMP1_Pin, &hspi3) != 0x00000011){//do the init and if failure report it to the error variable + put on led off
		err |= 0x00000100;
	}
	if(init_bmp280(NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2) != 0x00000011){//do the init and if failure report it to the error variable + put on led off
		err |= 0x00001000;
	}
	if(init_lis(NSS_LIS2_GPIO_Port, NSS_LIS2_Pin) != 0x00000011){//do the init and if failure report it to the error variable + put on led off
		err |= 0x00010000;
	}
	readParamBmp(Bmp1, NSS_BMP1_GPIO_Port, NSS_BMP1_Pin, &hspi3);
	readParamBmp(Bmp2, NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2);
	if(err != 0){HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);}
	return err;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t IMU1_data_raw[7] = {0}; 		//acc_temp_gyro
  float IMU1_data_converted[7] = {0}; 	//acc_temp_gyro
  uint16_t IMU2_data_raw[7] = {0}; 		//acc_temp_gyro
  float IMU2_data_converted[7] = {0}; 	//acc_temp_gyro
  int16_t pressure_pitot = 0;
  int32_t bmp1Raw[2] = {0}; 			//pressure-temp
  int32_t bmp1Comp[2] = {0}; 			//pressure-temp
  int32_t bmp2Raw[2] = {0}; 			//pressure-temp
  int32_t bmp2Comp[2] = {0}; 			//pressure-temp
  uint16_t LIS_data_raw[3] = {0};		//X-Y-Z
  float LIS_data_converted[3] = {0};	//X-Y-Z
  unsigned int err = 0;
  param Bmp1, Bmp2;						//compensation parameters
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //initialize NSS pin high for SPI communication
  HAL_GPIO_WritePin(NSS_ICM1_GPIO_Port, NSS_ICM1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NSS_ICM2_GPIO_Port, NSS_ICM2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NSS_BMP1_GPIO_Port, NSS_BMP1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NSS_NEO2_GPIO_Port, NSS_NEO2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);				//Put the LED on
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
  HAL_Delay(1000);
  err = mainInitialization(&Bmp1, &Bmp2);								//Configure the sensors
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	read16N(ICM20602_DATA_REG, &IMU1_data_raw, 7, NSS_ICM1_GPIO_Port, NSS_ICM1_Pin, &hspi3); //Read ICM data registers
	read16N(ICM20602_DATA_REG, &IMU2_data_raw, 7, NSS_ICM2_GPIO_Port, NSS_ICM2_Pin, &hspi2); //Read ICM data registers

	//Convert data in usual units (but float instead of unint16_t
	IMU1_data_converted[0] = (int16_t)IMU1_data_raw[0] / 2048.0; 			//in g
	IMU1_data_converted[1] = (int16_t)IMU1_data_raw[1] / 2048.0;
	IMU1_data_converted[2] = (int16_t)IMU1_data_raw[2] / 2048.0;
	IMU1_data_converted[3] = ((int16_t)IMU1_data_raw[3] / 326.8) + 25.0;	//in °C
	IMU1_data_converted[4] = (int16_t)IMU1_data_raw[4] / 65.5;				//in dps
	IMU1_data_converted[5] = (int16_t)IMU1_data_raw[5] / 65.5;
	IMU1_data_converted[6] = (int16_t)IMU1_data_raw[6] / 65.5;

	IMU2_data_converted[0] = (int16_t)IMU2_data_raw[0] / 2048.0; 			//in g
	IMU2_data_converted[1] = (int16_t)IMU2_data_raw[1] / 2048.0;
	IMU2_data_converted[2] = (int16_t)IMU2_data_raw[2] / 2048.0;
	IMU2_data_converted[3] = ((int16_t)IMU2_data_raw[3] / 326.8) + 25.0;	//in °C
	IMU2_data_converted[4] = (int16_t)IMU2_data_raw[4] / 65.5;				//in dps
	IMU2_data_converted[5] = (int16_t)IMU2_data_raw[5] / 65.5;
	IMU2_data_converted[6] = (int16_t)IMU2_data_raw[6] / 65.5;

	readBMP(BMP280_DATA_REG, &bmp1Raw, 2, NSS_BMP1_GPIO_Port, NSS_BMP1_Pin, &hspi3); //Read BMP data registers
	readBMP(BMP280_DATA_REG, &bmp2Raw, 2, NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2); //Read BMP data registers
	bmpCompensate(Bmp1, bmp1Raw, &bmp1Comp); //Compensate value of pressure and temperature with the compensation parameters
	bmpCompensate(Bmp2, bmp2Raw, &bmp2Comp); //Compensate value of pressure and temperature with the compensation parameters

	read16N(LIS_DATA_REG, &LIS_data_raw, 3, NSS_LIS2_GPIO_Port, NSS_LIS2_Pin, &hspi2); //Read LIS data registers
	//Convert data in usual units
	LIS_data_converted[0] = (int16_t)LIS_data_raw[0] / 6842.0; 			//in Gauss
	LIS_data_converted[1] = (int16_t)LIS_data_raw[1] / 6842.0;
	LIS_data_converted[2] = (int16_t)LIS_data_raw[2] / 6842.0;

	//pressure_pitot = readPitot();
	//HAL_Delay(1000);
	HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NSS_BMP1_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_BMP2_Pin|NSS_LIS2_Pin|NSS_ICM2_Pin|NSS_NEO2_Pin 
                          |NSS_ICM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_BMP1_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = NSS_BMP1_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC4 PC5 
                           PC6 PC7 PC8 PC9 
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_BMP2_Pin NSS_LIS2_Pin NSS_ICM2_Pin NSS_NEO2_Pin 
                           NSS_ICM1_Pin */
  GPIO_InitStruct.Pin = NSS_BMP2_Pin|NSS_LIS2_Pin|NSS_ICM2_Pin|NSS_NEO2_Pin 
                          |NSS_ICM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
