/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "readWrite.h"
#include "IMU.h"
#include "BMP.h"
#include "MAG.h"
#include "PITOT.h"
#include "apoapsis_detection.h"
#include "ublox_gps.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t arm;
extern uint32_t launch;
extern uint32_t sample;
extern uint32_t err_msg;
extern uint32_t err_init;
extern TIM_HandleTypeDef htim2;


typedef struct {
//	dateTimeStampParamTypeDef dateTime;			//key value pair of date time stamp
	float currentLatitude;
	float currentLongitude;
	float currentAltitude;
	float pDOP;
	float hDOP;
	float vDOP;
	float magneticHeading;
	float groundSpeedKmph;
	uint8_t fixParamters;
} gpsParsedPacketTypeDef1;

struct arrayRawFrame{
	  //uint8_t 	IMU3[WATERMARK_IMU3];						//MSB first		(500Hz)
	  //uint8_t 	IMU3[1008];
	  uint8_t 	FrameNumber;								//MSB first
	  uint8_t	Status1;									//MSB first
	  uint8_t 	Status2;									//MSB first
	  uint8_t 	Err_msg;									//LSB first
	  uint8_t 	RTC_field[4];								//MSB first
	  uint32_t 	Timer;										//LSB first
	  uint16_t  Battery1;									//LSB first
	  uint16_t 	Battery2;									//LSB first
	  uint8_t 	IMU2[WATERMARK_IMU2];						//MSB first		(200Hz)
	  int32_t 	BMP2[2];									//LSB first
	  int32_t 	BMP3[2];									//LSB first
	  uint8_t 	MAG[6];										//MSB first
	  uint8_t 	PITOT[2];									//LSB first
	  uint8_t	Endline1[2];								//MSB first
	  gpsParsedPacketTypeDef1	GPS; 						//GPS_struc defined by Sonal
	  uint8_t	Endline2[2];								//MSB first
	  uint8_t 	dummy[1008];								//read FIFO IMU avoid overflow structure
};
typedef struct arrayRawFrame ArrayRaw;

struct arrayConvFrame{
	  //float IMU3[WATERMARK_IMU3/2];						//acc_temp_gyro * 10 	(500Hz)
	  float IMU2[WATERMARK_IMU2/2];							//acc_temp_gyro * 4		(200Hz)
	  float BMP2[2];										//temp-pressure  		(50Hz)
	  float BMP3[2];										//temp-pressure  		(50Hz)
	  float MAG[3];											//X-Y-Z			  		(50Hz)
	  float PITOT;											//pressure  			(50Hz)
};
typedef struct arrayConvFrame ArrayConv;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
unsigned int sensorsInitialization(param *Bmp2, param *Bmp3);
void readSensors(ArrayRaw *Array);
void convertSensors(ArrayConv *ArrayConverted, ArrayRaw *ArrayToConvert);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NSS_BMP3_Pin GPIO_PIN_13
#define NSS_BMP3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOC
#define LIFTOFF_UMB_Pin GPIO_PIN_1
#define LIFTOFF_UMB_GPIO_Port GPIOA
#define EN_TELEM_Pin GPIO_PIN_2
#define EN_TELEM_GPIO_Port GPIOA
#define EN_FPV_Pin GPIO_PIN_3
#define EN_FPV_GPIO_Port GPIOA
#define UMBIL1_TELEM_Pin GPIO_PIN_4
#define UMBIL1_TELEM_GPIO_Port GPIOA
#define SPEED_TEST_Pin GPIO_PIN_7
#define SPEED_TEST_GPIO_Port GPIOA
#define UMBIL2_BUZZER_Pin GPIO_PIN_4
#define UMBIL2_BUZZER_GPIO_Port GPIOC
#define VBAT2_Pin GPIO_PIN_0
#define VBAT2_GPIO_Port GPIOB
#define VBAT1_Pin GPIO_PIN_1
#define VBAT1_GPIO_Port GPIOB
#define BOOT2_Pin GPIO_PIN_2
#define BOOT2_GPIO_Port GPIOB
#define INT_IMU3_Pin GPIO_PIN_8
#define INT_IMU3_GPIO_Port GPIOA
#define NSS_BMP2_Pin GPIO_PIN_9
#define NSS_BMP2_GPIO_Port GPIOA
#define NSS_MAG_Pin GPIO_PIN_10
#define NSS_MAG_GPIO_Port GPIOA
#define NSS_IMU2_Pin GPIO_PIN_11
#define NSS_IMU2_GPIO_Port GPIOA
#define NSS_GPS_Pin GPIO_PIN_12
#define NSS_GPS_GPIO_Port GPIOA
#define NSS_IMU3_Pin GPIO_PIN_15
#define NSS_IMU3_GPIO_Port GPIOA
#define EJEC_STM_Pin GPIO_PIN_8
#define EJEC_STM_GPIO_Port GPIOB
#define ARD_STATUS_Pin GPIO_PIN_9
#define ARD_STATUS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ERR_INIT_IMU2					1<<0;
#define ERR_INIT_IMU3					1<<1;
#define ERR_INIT_BMP2					1<<2;
#define ERR_INIT_BMP3					1<<3;
#define ERR_INIT_MAG					1<<4;
#define ERR_INIT_ADC1					1<<5;
#define ERR_INIT_ADC2					1<<6;
#define ERR_INIT_SD_CARD				1<<7;

#define ERR_LOOP_TIME					1<<0;
#define ERR_WRITE_SD					1<<1;
#define	ERR_SYNC_SD						1<<2;
#define ERR_SEND_TELEM					1<<3;
#define ERR_NUMBER_BYTES_IMU2			1<<4;

#define STATUS_TELEM					0;
#define STATUS_LAUNCHED					1;
#define STATUS_APOGEE					2;
#define STATUS_IGNITING					3;
#define STATUS_ARDUINO_ARMING			4;
#define STATUS_GROUND					5;


#define SIZEWITHGPS						138
#define SIZEWITHOUTGPS					98
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
