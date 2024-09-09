/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Block Segment
uint16_t firstSeg[] = {0xfe, 0xfe, 0x35, 0x31, 0x2d, 0x29, 0x25, 0x21, 0x1d, 0x19, 0x15, 0x11, 0x0d, 0x09, 0x05, 0x01};
uint16_t secSeg[] 	= {0xfe, 0xfe, 0x34, 0x30, 0x2c, 0x28, 0x24, 0x20, 0x1c, 0x18, 0x14, 0x10, 0x0c, 0x08, 0x04, 0x00};

// Variables for time
uint8_t jam = 0, menit = 0, detik = 0;
char time[10];
char date[10];
RTC_DateTypeDef gDate; 
RTC_TimeTypeDef gTime;
char Rx_deviceSerial[10], Tx_deviceSerial[10];
_Bool timeCmd, playCmd, wifiCmd;
char commandData[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// Function for sending data to HT1632C
void delay_us (uint16_t us);
void CS();
void CMD_100_Command(uint16_t byte);
void CMD_101_Write(uint16_t byte, uint8_t data);
void clear101();
void init();
uint16_t convertNumber(int num);

// Function for timer
void displayHour(uint8_t hr);
void displayMinSec(uint8_t min, uint8_t sec);
void sendToMonitor(char data[10]);
void sendToNucleog031(char data[10]);
void setTime(void);
void get_time(void);
void clearBuf();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	HAL_UART_Receive_IT(serialG031, (uint8_t *)Rx_deviceSerial, sizeof(Rx_deviceSerial)); 
	//sendToMonitor(Rx_deviceSerial);
}

void get_time(void) 
{  
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); 
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); 
	sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds); 
	//sendToNucleog031(time);
	//sendToMonitor(time);
	//sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year); 
}

void clearBuf() {for(int i=0; i<strlen(commandData); i++) {commandData[i] = '\0';}}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);			// Start timer 1 for delay_Us function
	init();													// Initialize display
	HAL_UART_Receive_IT(serialG031, (uint8_t *)Rx_deviceSerial, sizeof(Rx_deviceSerial)); 	// Receive interrupt data from UART
	
	// Init variable
	timeCmd 	= 1;
	playCmd 	= 1;
	wifiCmd		= 1;
	
	// Set time
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32f2)
	{
		setTime();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// Current time from RTC
		get_time();
		
		// Assign time to Transmit data buffer
		strncpy(Tx_deviceSerial, time, 10);
		// Assign received data from UART Nuclog031 to buffer
		strncpy(commandData, Rx_deviceSerial, 10);
		
		timeCmd 	= strcmp(commandData, "\ntime\r");
		playCmd 	= strcmp(commandData, "\nplay\r");
		wifiCmd		= strcmp(commandData, "\nwifi\r");
		
		// Send data time to other if requested
		if(timeCmd == 0)				
		{
			//sendToMonitor(Tx_deviceSerial);
			sendToNucleog031(Tx_deviceSerial); 
			HAL_Delay(100); 
			clearBuf();
			timeCmd = 1;
		}
		else if (playCmd == 0)	{CMD_101_Write(0x22, 0x1); playCmd = 1;}		// Turn On Play Symbol
		else if (wifiCmd == 0)	{CMD_101_Write(0x12, 0x1); wifiCmd = 1;}		// Turn On WiFi Symbol
		
		CMD_101_Write(0xe, 0x2);			// Titik dua kiri
		CMD_101_Write(0x12, 0x2);			// Titik dua kanan

		displayMinSec(gTime.Minutes, gTime.Seconds);
		displayHour(gTime.Hours);
		
		detik++;
		if (detik == 60)
		{
			detik = 0;
			menit++;
		}
		else if (menit == 60)
		{
			detik = 0;
			menit = 0;
			jam++;
		}
		else if (jam == 24)
		{
			jam = 0;
			menit = 0;
			detik++;
		}
		HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|Data_Pin|Write_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Read_GPIO_Port, Read_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin Data_Pin Write_Pin */
  GPIO_InitStruct.Pin = CS_Pin|Data_Pin|Write_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Read_Pin */
  GPIO_InitStruct.Pin = Read_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Read_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void sendToMonitor(char data[10])
{
	char buf[10];
	sprintf(buf, "\n%s\r", data);
	HAL_UART_Transmit_IT(serialMonitor, (uint8_t *)&buf, sizeof(buf));
}

void sendToNucleog031(char data[10])
{
	char buf[10];
	sprintf(buf, "\n%s\r", data);
	HAL_UART_Transmit_IT(serialG031, (uint8_t *)&buf, sizeof(buf));
}

void setTime(void)
{
	/* USER CODE BEGIN Check_RTC_BKUP */
	RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  /* USER CODE END Check_RTC_BKUP */
//
  /** Initialize RTC and set the Time and Date **/
  sTime.Hours = 0x10;
  sTime.Minutes = 0x17;
  sTime.Seconds = 0x00;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 0x19;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
  /* USER CODE END RTC_Init 2 */
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit_IT(serialG031, (uint8_t *)&ch, 1);
	HAL_UART_Transmit_IT(serialMonitor, (uint8_t *)&ch, 1);
  return ch;
}

void displayHour(uint8_t hr)
{
	uint8_t value;
	uint16_t puluhanJam = convertNumber((hr/10)%10);
	uint16_t satuanJam = convertNumber(hr%10);
	uint16_t charM = Char_M;
	uint16_t charA = Char_A;
	uint16_t charP = Char_P;
	
	for(int i=0; i<16; i++)
	{
		if 							(satuanJam&0x01) 			{value |= 1;}
		else 																	{value |= 0;}
		value <<=3;
		
		if 							(puluhanJam&0x01) 		{value |= 4;}
		else 																	{value |= 0;}
		
		if 							(charM&0x01) 					{value |= 2;}
		else																	{value |= 0;}
		
		if (hr >= 12)
		{	if 							(charP&0x01) 					{value |= 1;}
			else																	{value |= 0;}	}
		else if (hr < 12)
		{	if 							(charA&0x01) 					{value |= 1;}
			else																	{value |= 0;}	}
		
		
		CMD_101_Write(secSeg[i], value);
		charM >>= 1;
		charA >>= 1;
		charP >>= 1;
		puluhanJam >>= 1;
		satuanJam >>= 1;
		value = 0;
	}
}

void displayMinSec(uint8_t min, uint8_t sec)
{
	uint8_t value;			// Data will be send to address
	uint16_t puluhanMenit = convertNumber((min/10)%10);
	uint16_t satuanMenit = convertNumber(min%10);
	uint16_t puluhanDetik = convertNumber((sec/10)%10);
	uint16_t satuanDetik = convertNumber(sec%10);
	
	for (int i=0; i<16; i++)
	{
		if 							(satuanDetik&0x01)		{value |= 1;} 
		else 																	{value |= 0;}
		value <<= 1;
		
		if 							(puluhanDetik&0x01) 	{value |= 1;}
		else 																	{value |= 0;}
		value <<=1;
		
		if 							(satuanMenit&0x01) 		{value |= 1;}
		else 																	{value |= 0;}
		value <<=1;
		
		if 							(puluhanMenit&0x01) 	{value |= 1;}
		else 																	{value |= 0;}
		
		CMD_101_Write(firstSeg[i], value);
		puluhanMenit >>= 1;
		satuanMenit >>= 1;
		puluhanDetik >>= 1;
		satuanDetik >>= 1;
		value = 0;
	}
}

uint16_t convertNumber(int num)
{
	uint16_t converted;
	switch (num)
	{
		case (0x00):
			converted = ZERO;
			break;
		case (0x01):
			converted = ONE;
			break;
		case (0x02):
			converted = TWO;
			break;
		case (0x03):
			converted = THREE;
			break;
		case (0x04):
			converted = FOUR;
			break;
		case (0x05):
			converted = FIVE;
			break;
		case (0x06):
			converted = SIX;
			break;
		case (0x07):
			converted = SEVEN;
			break;
		case (0x08):
			converted = EIGHT;
			break;
		case (0x09):
			converted = NINE;
			break;
	}
	return converted;
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void CS()
{
	CS1;
	delay_us(50);
	CS0;
	delay_us(50);
}

void CMD_100_Command(uint16_t byte)
{
	uint8_t id = 0x04;
	
	CS();
	for (int i=0; i<3; i++)
	{
		writeClk0;
		if ((id&0x04) == 0x04){writePin(GPIOA, GPIO_PIN_1, 1);}
		else {writePin(GPIOA, GPIO_PIN_1, 0);}
		delay_us(50);
		writeClk1;
		delay_us(50);
		id = id >> 1;
	}
	
	for (int i=0; i<9; i++)
	{
		writeClk0;
		if ((byte & 0x100) ==  0x100)
		{
			writePin(GPIOA, GPIO_PIN_1, 1);
		}
		else
		{
			writePin(GPIOA, GPIO_PIN_1, 0);
		}
		delay_us(50);
		writeClk1;
		byte <<= 1;
		delay_us(50);
	}
}

void CMD_101_Write(uint16_t byte, uint8_t data)
{
	uint8_t id = 0x05;
	
	CS();
	for (int i=0; i<3; i++)
	{
		writeClk0;
		if ((id&0x01) == 0x01){writePin(GPIOA, GPIO_PIN_1, 1);}
		else {writePin(GPIOA, GPIO_PIN_1, 0);}
		delay_us(50);
		writeClk1;
		delay_us(50);
		id = id >> 1;
	}
	
	for (int i=0; i<7; i++)
	{
		writeClk0;
		if ((byte&0x40) ==  0x40)
		{
			writePin(GPIOA, GPIO_PIN_1, 1);
		}
		else
		{
			writePin(GPIOA, GPIO_PIN_1, 0);
		}
		delay_us(50);
		writeClk1;
		byte <<= 1;
		delay_us(50);
	}
	
	for (int i=0; i<4; i++)
	{
		writeClk0;
		if ((data&0x01) == 0x01)
		{
			writePin(GPIOA, GPIO_PIN_1, 1);
		}
		else
		{
			writePin(GPIOA, GPIO_PIN_1, 0);
		}
		delay_us(50);
		writeClk1;
		data = data >> 1;
		delay_us(50);
	}
}

void clear101()
{
	for(int f = 0; f < 95; f++){
		CMD_101_Write(f, 0x0);
	}
}

void init()
{
	CMD_100_Command(sys_dis);
	CMD_100_Command(N_Mos_16COM);
	CMD_100_Command(MasterMode);
	CMD_100_Command(sys_en);
	CMD_100_Command(pwmMax);
	CMD_100_Command(blinkOff);
	CMD_100_Command(led_on);
	clear101();
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
