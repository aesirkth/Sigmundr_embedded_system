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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t arm;
extern uint32_t launch;
extern uint32_t sample;
extern uint32_t err_msg;
extern TIM_HandleTypeDef htim2;

struct arrayRawFrame{
	  int16_t IMU2[70];										//acc_temp_gyro * 10 	(500Hz)
	  int16_t IMU3[28];										//acc_temp_gyro * 4		(200Hz)
	  int32_t BMP2[2];										//temp-pressure  		(50Hz)
	  int32_t BMP3[2];										//temp-pressure  		(50Hz)
	  int16_t MAG[3];										//X-Y-Z			  		(50Hz)
	  uint16_t PITOT;										//pressure  			(50Hz)
};
typedef struct arrayRawFrame ArrayRaw;

struct arrayConvFrame{
	  float IMU2[70];										//acc_temp_gyro * 10 	(500Hz)
	  float IMU3[28];										//acc_temp_gyro * 4		(200Hz)
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
ArrayRaw readSensors(void);
ArrayConv convertSensors(ArrayRaw ArrayToConvert);
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
#define LIFTOFF_UMB_EXTI_IRQn EXTI1_IRQn
#define TELEM_Pin GPIO_PIN_2
#define TELEM_GPIO_Port GPIOA
#define EN_TELEM_UMBIL1_Pin GPIO_PIN_4
#define EN_TELEM_UMBIL1_GPIO_Port GPIOA
#define CALIB_UMBIL2_Pin GPIO_PIN_4
#define CALIB_UMBIL2_GPIO_Port GPIOC
#define UMBIL3_Pin GPIO_PIN_5
#define UMBIL3_GPIO_Port GPIOC
#define VBAT2_Pin GPIO_PIN_0
#define VBAT2_GPIO_Port GPIOB
#define VBAT1_Pin GPIO_PIN_1
#define VBAT1_GPIO_Port GPIOB
#define BOOT2_Pin GPIO_PIN_2
#define BOOT2_GPIO_Port GPIOB
#define INT_IMU3_Pin GPIO_PIN_8
#define INT_IMU3_GPIO_Port GPIOA
#define INT_IMU3_EXTI_IRQn EXTI9_5_IRQn
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
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
