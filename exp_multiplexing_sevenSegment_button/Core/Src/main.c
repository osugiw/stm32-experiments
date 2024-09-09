/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "seven_segment.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Button PTD
typedef struct{
	uint8_t readState;
	uint8_t currentState;
	uint8_t lastState;
	uint32_t lastDebounceTime;
} button_t;

// Button state for controlling system state
typedef struct{
	bool startCooking;
	bool startWarming;
	bool selectMenu;
	bool enableChangeTime;
	bool incrementHour;
	bool incrementMinute;
}buttonState_t;

// System state
typedef struct{
	bool isCooking;					// Cooking state
	bool isWarming;					// Warming state
	bool isChangeTime;				// State for configuring time (preset timer and schedule timer)
	uint8_t currentMenu;			// Current menu selection to determine the preset of timer and temperature
} systemState_t;

// System Timer variables
typedef struct{
	// Placed in the interrupt callback
	volatile uint32_t testCounter;
	uint32_t newCookingTimer;			// Temporary Timer for adjusted preset timer
} systemTimer_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t _msgHalo[4] 	= {0x68, 0x61, 0x6C, 0x6F};						// Greeting message
const uint8_t _msgDone[4]	= {0x64, 0x6F, 0x6E, 0x65};						// Finished cooking message

buttonState_t buttonState 	= {false, false, false, false, false, false};
systemState_t systemState	= {false, false, false, false};
systemTimer_t systemTimer	= {false, false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void enableDisplay(void);
void enableButton(button_t *inputBT);
void controlButtonState(button_t *inputBT, uint32_t debounceDelay, bool *returnState);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 *	@brief	Blinking function for LEDs
 *	#param
 *		switchDelay		a struct that holds switching variables
 *		debounceDelay	time to delay
 *	@retval None
***/
void intervalSwitching(switching_delay_t *switchDelay, uint32_t debounceDelay)
{
	if(HAL_GetTick() - switchDelay->lastTime > debounceDelay){
		switchDelay->lastTime = HAL_GetTick();
		switchDelay->_state = !switchDelay->_state;
	}
}

void enableDisplay(void)
{
	// Turn off switch selector
	COM_SW_OFF;

	// Initialize the Seven Segment pins as the Output Pin
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin SEG_D_Pin SEG_E_Pin SEG_F_Pin SEG_G_Pin SEG_DOT_Pin */
	GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DOT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void enableButton(button_t *inputBT)
{
	// Turn off Display COMs
	blankDisplay();

	// Initialize the Seven Segment pins as the Input Pin
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pins : SW_COM_Pin sw_schedule_cook_Pin sw_white_rice_Pin sw_red_rice_Pin */
	GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin|SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DOT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	COM_SW_ON;

	// Read Button State
	inputBT[0].readState	= HAL_GPIO_ReadPin(SEG_A_GPIO_Port, SEG_A_Pin);
	inputBT[1].readState	= HAL_GPIO_ReadPin(SEG_B_GPIO_Port, SEG_B_Pin);
	inputBT[2].readState 	= HAL_GPIO_ReadPin(SEG_C_GPIO_Port, SEG_C_Pin);
	inputBT[3].readState	= HAL_GPIO_ReadPin(SEG_D_GPIO_Port, SEG_D_Pin);
	inputBT[4].readState 	= HAL_GPIO_ReadPin(SEG_E_GPIO_Port, SEG_E_Pin);
	inputBT[5].readState 	= HAL_GPIO_ReadPin(SEG_F_GPIO_Port, SEG_F_Pin);

	// Read button state
	controlButtonState(&inputBT[0], BUTTON_DEBOUNCE_DELAY, &buttonState.startWarming);
	controlButtonState(&inputBT[1], BUTTON_DEBOUNCE_DELAY, &buttonState.startCooking);
	controlButtonState(&inputBT[2], BUTTON_DEBOUNCE_DELAY, &buttonState.enableChangeTime);
	controlButtonState(&inputBT[3], BUTTON_DEBOUNCE_DELAY, &buttonState.selectMenu);
	controlButtonState(&inputBT[4], BUTTON_DEBOUNCE_DELAY, &buttonState.incrementHour);
	controlButtonState(&inputBT[5], BUTTON_DEBOUNCE_DELAY, &buttonState.incrementMinute);

	// Control System State
	if(buttonState.enableChangeTime){
		systemState.isChangeTime = !systemState.isChangeTime;
		buttonState.enableChangeTime = false;
	}
	// Start Cooking mode State
	else if(buttonState.startCooking){
		systemState.isCooking			= true;
		systemState.isWarming			= false;
		buttonState.startCooking		= false;
	}
	// Start Cooking mode State
	else if(buttonState.startWarming){
		if(systemState.isCooking){
			systemState.isCooking = false;
		}
		else if(systemState.isWarming){
			systemState.isWarming = false;
		}
		else{
			systemState.isWarming = true;
		}
		buttonState.startWarming		= false;
	}
	// Change menus
	else if(buttonState.selectMenu){
		if(systemState.currentMenu < 9){
			systemState.currentMenu += 1;
		}
		else{
			systemState.currentMenu = 0;
		}
		buttonState.selectMenu = false;
	}

	// Adjust Preset Timer for Adjustable Mode Only
	if(systemState.isChangeTime){
		systemState.isCooking 	= false;
		systemState.isWarming 	= false;

		if(buttonState.incrementHour){
			systemTimer.newCookingTimer		+= 3600;
			buttonState.incrementHour 		= false;
		}

		if(buttonState.incrementMinute){
			systemTimer.newCookingTimer		+= 300;
			buttonState.incrementMinute 	= false;
		}
	}

	for(uint8_t i=0; i<6; i++){
		inputBT[i].lastState = inputBT[i].readState;
	}
}

/**
 * @brief Control the button state
 * @param
 * 		button				Struct of button's state
 * 		debounceDelay		button delay read
 * 		returnState			State to change when the button is pressed
 * @retval None
**/
void controlButtonState(button_t *inputBT, uint32_t debounceDelay, bool *returnState)
{
	// Ignoring noise or accidentally pressed
	if(inputBT->currentState != inputBT->lastState)
		inputBT->lastDebounceTime = HAL_GetTick();

	// Read the actual state from the button
	if((HAL_GetTick() - inputBT->lastDebounceTime) > debounceDelay){
		if(inputBT->readState != inputBT->currentState){
			inputBT->currentState = inputBT->readState;
			if(inputBT->currentState == GPIO_PIN_RESET){
				*returnState = true;
			}
		}
	}
}

/**
 * @brief Elapsed timer callback
 * @param:
 * 		htim	Timer peripheral
 * @retval None
**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim14.Instance)
	{
		systemTimer.testCounter += 1;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// Button variables
	button_t bt_startWarming	= {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0};
	button_t bt_startCooking	= {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0};
	button_t bt_selectMenu		= {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0};
	button_t bt_enChangeTime 	= {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0};
	button_t bt_changeHour		= {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0};
	button_t bt_changeMinute	= {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, 0};
	button_t arrButtons[6] = {bt_startWarming, bt_startCooking, bt_enChangeTime, bt_selectMenu, bt_changeHour, bt_changeMinute};
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
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Start Timer
  HAL_TIM_Base_Start(&htim1);						// Start timer 1 for delay_Us function
  HAL_TIM_Base_Start_IT(&htim14);					// Timer counter
  displayText(_msgHalo, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  enableDisplay();
	  displayTimer(systemTimer.testCounter, true, true);
	  enableButton(arrButtons);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
  htim3.Init.Prescaler = 64000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15000;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 64000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 64000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DOT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SW_COM_Pin|SEG_COM1_Pin|SEG_COM2_Pin|SEG_COM3_Pin
                          |SEG_COM4_Pin|SEG_COM5_Pin|SEG_COM6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin SEG_D_Pin
                           SEG_E_Pin SEG_F_Pin SEG_G_Pin SEG_DOT_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_COM_Pin SEG_COM1_Pin SEG_COM2_Pin SEG_COM3_Pin
                           SEG_COM4_Pin SEG_COM5_Pin SEG_COM6_Pin */
  GPIO_InitStruct.Pin = SW_COM_Pin|SEG_COM1_Pin|SEG_COM2_Pin|SEG_COM3_Pin
                          |SEG_COM4_Pin|SEG_COM5_Pin|SEG_COM6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 *	@brief	Non-blocking Delay in Microseconds
 *	@retval None
***/
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  					// set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
