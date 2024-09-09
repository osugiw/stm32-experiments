/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SEVEN_SEGMENT_H
#define __SEVEN_SEGMENT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
// ASCII Character
#define NOL 			0x3f
#define SATU 			0x06
#define DUA 			0x5b
#define TIGA 			0x4f
#define EMPAT 		0x66
#define LIMA 			0x6d
#define ENAM 			0x7d
#define TUJUH 		0x07
#define DELAPAN 	0x7f
#define SEMBILAN 	0x6f
#define CHAR_E		0x79
#define CHAR_R		0x50


// COM Selection
#define COM1_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define COM1_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define COM2_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define COM2_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define COM3_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define COM3_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define COM4_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define COM4_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

// Functions
uint8_t convertNumber(uint8_t num);
void checkSegment(void);

#ifdef __cplusplus
}
#endif

#endif /* __SEVEN_SEGMENT_H */