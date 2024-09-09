/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Struct for controlling display switching
typedef struct{
	uint32_t lastTime;
	bool _state;
} switching_delay_t;
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
void intervalSwitching(switching_delay_t *switchDelay, uint32_t debounceDelay);
void delay_us (uint16_t us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEG_A_Pin GPIO_PIN_4
#define SEG_A_GPIO_Port GPIOA
#define SEG_B_Pin GPIO_PIN_5
#define SEG_B_GPIO_Port GPIOA
#define SEG_C_Pin GPIO_PIN_6
#define SEG_C_GPIO_Port GPIOA
#define SEG_D_Pin GPIO_PIN_7
#define SEG_D_GPIO_Port GPIOA
#define SW_COM_Pin GPIO_PIN_1
#define SW_COM_GPIO_Port GPIOB
#define SEG_E_Pin GPIO_PIN_8
#define SEG_E_GPIO_Port GPIOA
#define SEG_F_Pin GPIO_PIN_9
#define SEG_F_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_7
#define buzzer_GPIO_Port GPIOC
#define SEG_G_Pin GPIO_PIN_10
#define SEG_G_GPIO_Port GPIOA
#define SEG_DOT_Pin GPIO_PIN_11
#define SEG_DOT_GPIO_Port GPIOA
#define T_JTMS_Pin GPIO_PIN_13
#define T_JTMS_GPIO_Port GPIOA
#define T_JTCK_Pin GPIO_PIN_14
#define T_JTCK_GPIO_Port GPIOA
#define SEG_COM1_Pin GPIO_PIN_3
#define SEG_COM1_GPIO_Port GPIOB
#define SEG_COM2_Pin GPIO_PIN_4
#define SEG_COM2_GPIO_Port GPIOB
#define SEG_COM3_Pin GPIO_PIN_5
#define SEG_COM3_GPIO_Port GPIOB
#define SEG_COM4_Pin GPIO_PIN_6
#define SEG_COM4_GPIO_Port GPIOB
#define SEG_COM5_Pin GPIO_PIN_7
#define SEG_COM5_GPIO_Port GPIOB
#define SEG_COM6_Pin GPIO_PIN_8
#define SEG_COM6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BUTTON_DEBOUNCE_DELAY 	20		// Debounce time
#define BLINK_DELAY				500
#define MAX_HOURS				64		// Maximum hour to display for the timer
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
