/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

// Declare Value to send
#define ZERO 		0xfc00
#define ONE 		0x6000
#define TWO			0xdb00
#define THREE		0xf300
#define FOUR		0x6700
#define FIVE		0xb700
#define SIX			0xbf00
#define SEVEN		0xe000
#define EIGHT		0xff00
#define NINE		0xf700
#define Char_A 	0xef00
#define Char_P	0xcf00
#define Char_M	0x6ca0

#define writePin HAL_GPIO_WritePin
#define CS0	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define CS1 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define writeClk0 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define writeClk1 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define readClk0 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define readClk1 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)


#define sys_dis 0x0

// COM
#define N_Mos_8COM 0x40
#define P_Mos_8COM 0x50
#define N_Mos_16COM 0x48
#define P_Mos_16COM 0x58

// System
#define MasterMode 0x30
#define sys_en 0x2
#define led_on 0x6
#define blinkOff 0x10
#define blinkOn 0x12

// PWM
#define pwmMax 0x015e

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOA
#define Data_Pin GPIO_PIN_1
#define Data_GPIO_Port GPIOA
#define Write_Pin GPIO_PIN_4
#define Write_GPIO_Port GPIOA
#define Read_Pin GPIO_PIN_1
#define Read_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
