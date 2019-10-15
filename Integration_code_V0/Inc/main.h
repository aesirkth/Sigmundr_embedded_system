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
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NSS_BMP3_Pin GPIO_PIN_13
#define NSS_BMP3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOC
#define TELEM_Pin GPIO_PIN_2
#define TELEM_GPIO_Port GPIOA
#define UMBIL1_Pin GPIO_PIN_4
#define UMBIL1_GPIO_Port GPIOA
#define UMBIL2_Pin GPIO_PIN_4
#define UMBIL2_GPIO_Port GPIOC
#define UMBIL3_Pin GPIO_PIN_5
#define UMBIL3_GPIO_Port GPIOC
#define VBAT2_Pin GPIO_PIN_0
#define VBAT2_GPIO_Port GPIOB
#define VBAT1_Pin GPIO_PIN_1
#define VBAT1_GPIO_Port GPIOB
#define BOOT2_Pin GPIO_PIN_2
#define BOOT2_GPIO_Port GPIOB
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