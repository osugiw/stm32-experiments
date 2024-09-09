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
#define NOL 0x3f
#define SATU 0x06
#define DUA 0x5b
#define TIGA 0x4f
#define EMPAT 0x66
#define LIMA 0x6d
#define ENAM 0x7d
#define TUJUH 0x07
#define DELAPAN 0x7f
#define SEMBILAN 0x6f

// Digits Settings
#define fourDigits 0x00
#define fiveDigits 0x01
#define sixDigits 0x02
#define sevenDigits 0x03

// Data Settings Command
#define fixedAddress 0x44
#define incrementAddress 0x40

// Address Setting Command
#define clearDisplay 0xc0
// DIGIT0 (LEFT) TO DIGIT13 (RIGHT)
#define digit1Seg0 0x00	// REC, RAND, PAUSE, USB, CARD, 0 ARROW:00.00, MIC L-R, MHZ, MUSIC, VIDEO
#define digit1Seg1 0x01	// REC, RAND, PAUSE, USB, CARD, 0:00.00, MIC L-R, MHZ, MUSIC, VIDEO
#define digit2Seg2 0x02	// REC, RAND, PAUSE, USB, CARD, 0 ARROW:00.00, MIC L-R, MHZ, MUSIC, VIDEO
#define digit2Seg3 0x03	// REC, RAND, PAUSE, USB, CARD, 0 ARROW:00.00, MIC L-R, MHZ, MUSIC, VIDEO
#define digit3seg4 0x04	// 0 ARROW 00.00
#define digit3Seg5 0x05	// 0 ARROW 00.00
#define digit4Seg6 0x06	// 0 ARROW 00.00.0
#define digit4Seg7 0x07	// REC, RAND, PLAY, PAUSE, USB, CARD, 0 ARROW:00.00, MIC L-R, BAZZ, MHZ, MUSIC, VIDEO
#define digit5Seg8 0x08 // 0 ARROW 00.00.0
#define digit5Seg9 0x09	// 0 ARROW 00.00.0
#define digit6SegA 0x0a // REC, RAND, PLAY, PAUSE, USB, CARD, 0 ARROW:00.00, MIC L-R, BAZZ, MHZ, MUSIC, VIDEO
#define digit6SegB 0x0b
#define digit7SegC 0x0c
#define digit7SegD 0x0d

// Brightness Setting Command
#define displayOff 0x80
#define brightnessLevel1 0x88
#define brightnessLevel2 0x89
#define brightnessLevel3 0x8a
#define brightnessLevel4 0x8b
#define brightnessLevel5 0x8c
#define brightnessLevel6 0x8d
#define brightnessLevel7 0x8e
#define brightnessLevel8 0x8f

#define writePin HAL_GPIO_WritePin
#define togglePin HAL_GPIO_TogglePin

#define clockLow HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0)
#define clockHigh HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1)

#define Strobe0 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0)
#define Strobe1 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1)

#define Power 0x8976E817
#define VolUp 0x89760AF5
#define VolDown 0x89768A75
#define Stop 0x8976C837
#define Skip 0x897658A7
#define Previous 0x8976D827

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
#define LED_STB_Pin GPIO_PIN_0
#define LED_STB_GPIO_Port GPIOA
#define LED_CLK_Pin GPIO_PIN_1
#define LED_CLK_GPIO_Port GPIOA
#define LED_DIO_Pin GPIO_PIN_4
#define LED_DIO_GPIO_Port GPIOA
#define BuiltIn_LED_Pin GPIO_PIN_5
#define BuiltIn_LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
