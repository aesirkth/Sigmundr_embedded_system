/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime = {0};								//time from RTC
param Bmp2, Bmp3;											//compensation parameters for BMP
uint32_t arm = 0;											//arm variable 	(extern)
uint32_t launch = 0;										//launch variable (extern)
uint32_t sample = 0;										//sample variable (extern)
uint32_t err_msg = 0;										//err_msg variable
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t readPinPlug = 0;
	uint32_t timerPostLaunch = 0;
	ArrayRaw DataRawArray = {0};
	ArrayRaw *pointDataRawArray = &DataRawArray;
	ArrayRaw DataRawArrayFreez = {0};
	ArrayRaw *pointDataRawArrayFreez = &DataRawArrayFreez;
	ArrayConv DataConvArray = {{0.0}};
	ArrayRaw *pointDataConvArray = &DataConvArray;
	uint32_t epoch = 0;
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
   //HAL_TIM_Base_Start_IT(&htim3);																//timer for the sampling
   err_msg |= sensorsInitialization(&Bmp2, &Bmp3);												//Initialization of sensors (IMU,BMP,MAG)
   if(HAL_ADC_Start_DMA(&hadc1, (uint16_t*) pointDataRawArray->Battery1, 1) != HAL_OK){err_msg |= ERR_INIT_ADC;}		//Initialization ADC1 (reading voltage Vbat1)
   if(HAL_ADC_Start_DMA(&hadc2, (uint16_t*) pointDataRawArray->Battery2, 1) != HAL_OK){err_msg |= ERR_INIT_ADC;}		//Initialization ADC2 (reading voltage Vbat2)
   //err_msg |= SD card initialization		----- ADD THIS -----

   /*
   while(readPinPlug == 0){																//wait till liftoff wire is connected
	   readPinPlug = 1;
	   for(uint32_t i=0; i<100; i++){												//make sure the connection is solid
			   readPinPlug &= HAL_GPIO_ReadPin(LIFTOFF_UMB_GPIO_Port, LIFTOFF_UMB_Pin);
			   HAL_Delay(10);
	   }
   }
   */
   clearFIFO(NSS_IMU3_GPIO_Port, NSS_IMU3_Pin, &hspi3);		//clear the FIFO of the IMU
   clearFIFO(NSS_IMU2_GPIO_Port, NSS_IMU2_Pin, &hspi2); 	//clear the FIFO of the IMU
   arm = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while(sample == 0); 												//Wait till new sample time (change by interrupt)
	  htim3.Instance->CNT = 0;											//put back the CNT from timer3 (for sampling) to zero
	  sample = 0;
	  epoch++;
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); 					//sTime.Hours .Minutes .Seconds .SubSeconds (up to 256).
	  timerPostLaunch = __HAL_TIM_GET_COUNTER(&htim2);					//Check time (counter) post launch

	  readIMU((uint8_t*) pointDataRawArray->IMU3, WATERMARK_IMU3, NSS_IMU3_GPIO_Port, NSS_IMU3_Pin, &hspi2);
	  readIMU((uint8_t*) pointDataRawArray->IMU2, WATERMARK_IMU2, NSS_IMU2_GPIO_Port, NSS_IMU2_Pin, &hspi3);

	  DataRawArray.RTC_field[0] = sTime.Hours;								//Put RTC in the frame
	  DataRawArray.RTC_field[1] = sTime.Minutes;
	  DataRawArray.RTC_field[2] = sTime.Seconds;
	  DataRawArray.RTC_field[3] = (uint8_t) sTime.SubSeconds;
	  DataRawArray.Timer = timerPostLaunch;

	  convertSensors((ArrayConv*)pointDataConvArray, (ArrayRaw*)pointDataRawArrayFreez);	//Convert the Data if needed (IMU-BMP-MAG-PITOT)
	  //Process GPS raw data to float

	  while(hspi2.State != HAL_SPI_STATE_READY){err_msg |= WAIT_IMU2_FINISH_BEFORE_GPS;}
	  if(epoch == 5){
		  epoch = 0;
		  DataRawArray.FrameNumber = 2;
		  //READ GPS DMA			----- ADD THIS -----
	  }
	  else{DataRawArray.FrameNumber = 1;}

	  //DETECT APOGEE FUNCTION		----- ADD THIS -----

	  HAL_ADC_Start(&hadc1);												//Read battery voltage1, put in Vbat1
	  HAL_ADC_Start(&hadc2);												//Read battery voltage2, put in Vbat2

	  readPITOT((uint8_t*) pointDataRawArray->PITOT, &hi2c1);

	  while(hspi3.State != HAL_SPI_STATE_READY){err_msg |= WAIT_IMU3_FINISH_BEFORE_BMP3;}
	  readBMPCal((int32_t*) pointDataRawArray->BMP3, Bmp3, NSS_BMP3_GPIO_Port, NSS_BMP3_Pin, &hspi3);
	  while(hspi2.State != HAL_SPI_STATE_READY){err_msg |= WAIT_GPS_FINISH_BEFORE_BMP2};
	  readBMPCal((int32_t*) pointDataRawArray->BMP2, Bmp2, NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2);
	  readMAG((uint8_t*) pointDataRawArray->MAG, NSS_MAG_GPIO_Port, NSS_MAG_Pin, &hspi2);
	  while((hdma_adc1.State != HAL_DMA_STATE_READY) && (hdma_adc2.State != HAL_DMA_STATE_READY)){err_msg |= WAIT_ADC_TO_FINISH};

	  DataRawArray.Err_msg = (uint16_t) err_msg;
	  err_msg &= RESET_ERR_MSG;
	  DataRawArrayFreez = DataRawArray;
	  memset(pointDataRawArray, 0, sizeof(DataRawArray));

	  //WRITE SD CARD DMA			----- ADD THIS -----
	  //SEND TELEM DMA				----- ADD THIS -----
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SDIO
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 21000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  HAL_GPIO_WritePin(NSS_BMP3_GPIO_Port, NSS_BMP3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|UMBIL3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TELEM_Pin|SPEED_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_BMP2_Pin|NSS_MAG_Pin|NSS_IMU2_Pin|NSS_GPS_Pin 
                          |NSS_IMU3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EJEC_STM_GPIO_Port, EJEC_STM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_BMP3_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = NSS_BMP3_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LIFTOFF_UMB_Pin */
  GPIO_InitStruct.Pin = LIFTOFF_UMB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIFTOFF_UMB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TELEM_Pin SPEED_TEST_Pin */
  GPIO_InitStruct.Pin = TELEM_Pin|SPEED_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_TELEM_UMBIL1_Pin */
  GPIO_InitStruct.Pin = EN_TELEM_UMBIL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN_TELEM_UMBIL1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CALIB_UMBIL2_Pin */
  GPIO_InitStruct.Pin = CALIB_UMBIL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CALIB_UMBIL2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UMBIL3_Pin */
  GPIO_InitStruct.Pin = UMBIL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(UMBIL3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT2_Pin */
  GPIO_InitStruct.Pin = BOOT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_IMU3_Pin */
  GPIO_InitStruct.Pin = INT_IMU3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_IMU3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_BMP2_Pin NSS_MAG_Pin NSS_IMU2_Pin NSS_GPS_Pin 
                           NSS_IMU3_Pin */
  GPIO_InitStruct.Pin = NSS_BMP2_Pin|NSS_MAG_Pin|NSS_IMU2_Pin|NSS_GPS_Pin 
                          |NSS_IMU3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EJEC_STM_Pin */
  GPIO_InitStruct.Pin = EJEC_STM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EJEC_STM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
// Initialization of the sensors, report error if any
unsigned int sensorsInitialization(param *Bmp2, param *Bmp3){
	unsigned int err = 0;
	if(initIMU_slow(NSS_IMU2_GPIO_Port, NSS_IMU2_Pin, &hspi2) != 1){err |= ERR_INIT_IMU2;}
	if(initIMU_fast(NSS_IMU3_GPIO_Port, NSS_IMU3_Pin, &hspi3) != 1){err |= ERR_INIT_IMU3;}
	if(initBMP(		NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2) != 1){err |= ERR_INIT_BMP2;}
	if(initBMP(		NSS_BMP3_GPIO_Port, NSS_BMP3_Pin, &hspi3) != 1){err |= ERR_INIT_BMP3;}
	if(initMAG(		NSS_MAG_GPIO_Port , NSS_MAG_Pin , &hspi2) != 1){err |= ERR_INIT_MAG;}

	readParamBmp(Bmp2, NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2);
	readParamBmp(Bmp3, NSS_BMP3_GPIO_Port, NSS_BMP3_Pin, &hspi3);

	if(err != 0){HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);}
	return err;
}

void readSensors(ArrayRaw *Array){
	readIMU((uint8_t*) Array->IMU3, 140	, NSS_IMU3_GPIO_Port, NSS_IMU3_Pin, &hspi2);
	readIMU((uint8_t*) Array->IMU2, 56	, NSS_IMU2_GPIO_Port, NSS_IMU2_Pin, &hspi3);
	readBMPCal((int32_t*) Array->BMP2, Bmp2, NSS_BMP2_GPIO_Port, NSS_BMP2_Pin, &hspi2);
	readBMPCal((int32_t*) Array->BMP3, Bmp3, NSS_BMP3_GPIO_Port, NSS_BMP3_Pin, &hspi3);
	readMAG((uint8_t*) Array->MAG, NSS_MAG_GPIO_Port, NSS_MAG_Pin, &hspi2);
	readPITOT((uint8_t*) Array->PITOT, &hi2c1);
}

void convertSensors(ArrayConv *ArrayConverted, ArrayRaw *ArrayToConvert){
	uint32_t cycleIMU3 = 10;
	uint32_t cycleIMU2 = 4;

	convIMU((uint8_t*) ArrayToConvert->IMU2, (float*) ArrayConverted->IMU2, cycleIMU2);
	convIMU((uint8_t*) ArrayToConvert->IMU3, (float*) ArrayConverted->IMU3, cycleIMU3);
	convBMP((int32_t*) ArrayToConvert->BMP2, (float*) ArrayConverted->BMP2);
	convBMP((int32_t*) ArrayToConvert->BMP3, (float*) ArrayConverted->BMP3);
	convMAG((uint8_t*) ArrayToConvert->MAG , (float*) ArrayConverted->MAG );
	convSpeedPITOT((uint8_t*) &ArrayToConvert->PITOT, (float*) &ArrayConverted->PITOT);
}

// Put back the NSS pin high. Since we don't know which sensor was read we put them all high
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi2){
		HAL_GPIO_WritePin(NSS_IMU2_GPIO_Port, NSS_IMU2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(NSS_GPS_GPIO_Port , NSS_GPS_Pin , GPIO_PIN_SET);
	}
	else if(hspi == &hspi3){
		HAL_GPIO_WritePin(NSS_IMU3_GPIO_Port, NSS_IMU3_Pin, GPIO_PIN_SET);
	}
}

void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma){
	__NOP();		//for debugging
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	__NOP();		//for debugging
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	if		(hspi == &hspi2){err_msg |= ERR_SPI2_ERRORCALLBACK;}
	else if	(hspi == &hspi3){err_msg |= ERR_SPI3_ERRORCALLBACK;}
}

void ADC_DMAError(DMA_HandleTypeDef *hdma){
	if		(hdma == &hdma_adc1){err_msg |= ERR_ADC_ERRORCALLBACK;}
	else if	(hdma == &hdma_adc2){err_msg |= ERR_ADC_ERRORCALLBACK;}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	err_msg |= ERR_UART_ERRORCALLBACK;
}

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
