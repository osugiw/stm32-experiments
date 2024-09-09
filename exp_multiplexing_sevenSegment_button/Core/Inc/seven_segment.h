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
#define COM_SWITCH_DELAY	3000

// ASCII Character
#define NOL 		0x3f
#define SATU 		0x06
#define DUA 		0x5b
#define TIGA 		0x4f
#define EMPAT 		0x66
#define LIMA 		0x6d
#define ENAM 		0x7d
#define TUJUH 		0x07
#define DELAPAN 	0x7f
#define SEMBILAN 	0x6f
#define CHAR_a		0x5F
#define CHAR_C		0x39
#define CHAR_d		0x5E
#define CHAR_e		0X7B
#define CHAR_E		0x79
#define CHAR_h		0x76
#define CHAR_n		0x54
#define CHAR_R		0x50
#define CHAR_P		0x73


// COM Selection & Dot
#define COM1_ON			HAL_GPIO_WritePin(SEG_COM1_GPIO_Port, SEG_COM1_Pin , GPIO_PIN_SET)
#define COM1_OFF		HAL_GPIO_WritePin(SEG_COM1_GPIO_Port, SEG_COM1_Pin , GPIO_PIN_RESET)
#define COM2_ON			HAL_GPIO_WritePin(SEG_COM2_GPIO_Port, SEG_COM2_Pin, GPIO_PIN_SET)
#define COM2_OFF		HAL_GPIO_WritePin(SEG_COM2_GPIO_Port, SEG_COM2_Pin, GPIO_PIN_RESET)
#define COM3_ON			HAL_GPIO_WritePin(SEG_COM3_GPIO_Port, SEG_COM3_Pin, GPIO_PIN_SET)
#define COM3_OFF		HAL_GPIO_WritePin(SEG_COM3_GPIO_Port, SEG_COM3_Pin, GPIO_PIN_RESET)
#define COM4_ON			HAL_GPIO_WritePin(SEG_COM4_GPIO_Port, SEG_COM4_Pin, GPIO_PIN_SET)
#define COM4_OFF		HAL_GPIO_WritePin(SEG_COM4_GPIO_Port, SEG_COM4_Pin, GPIO_PIN_RESET)
#define COM5_ON			HAL_GPIO_WritePin(SEG_COM5_GPIO_Port, SEG_COM5_Pin, GPIO_PIN_SET)
#define COM5_OFF		HAL_GPIO_WritePin(SEG_COM5_GPIO_Port, SEG_COM5_Pin, GPIO_PIN_RESET)
#define COM6_ON			HAL_GPIO_WritePin(SEG_COM6_GPIO_Port, SEG_COM6_Pin, GPIO_PIN_SET)
#define COM6_OFF		HAL_GPIO_WritePin(SEG_COM6_GPIO_Port, SEG_COM6_Pin, GPIO_PIN_RESET)
#define COM_SW_ON		HAL_GPIO_WritePin(SW_COM_GPIO_Port, SW_COM_Pin, GPIO_PIN_SET)
#define COM_SW_OFF		HAL_GPIO_WritePin(SW_COM_GPIO_Port, SW_COM_Pin, GPIO_PIN_RESET)
#define DOUBLE_DOT_ON	HAL_GPIO_WritePin(SEG_DOT_GPIO_Port, SEG_DOT_Pin, GPIO_PIN_SET)
#define DOUBLE_DOT_OFF	HAL_GPIO_WritePin(SEG_DOT_GPIO_Port, SEG_DOT_Pin, GPIO_PIN_RESET)
#define DOUBLE_DOT_TOG	HAL_GPIO_TogglePin(SEG_DOT_GPIO_Port, SEG_DOT_Pin)

// Functions
uint8_t convertValue(uint8_t num);
void checkSegment(void);
void writeSegment(uint8_t value, _Bool _useDot);
void blankDisplay(void);

// COM Selection
void digitCOM1(uint8_t _value);
void digitCOM2(uint8_t _value, _Bool _useDot);
void digitCOM3(uint8_t _value, _Bool _useDot);
void digitCOM4(uint8_t _value);
void digitCOM5(uint8_t _value);
void digitCOM6(uint8_t _value);

// Display function
void displayTimer(uint32_t inputSecond, _Bool blinkAll,_Bool hoursMinutesOnly);
void displayMode(uint8_t _mode);
void displayText(const uint8_t* _text, uint32_t _delay);
void displayLoading();
void displayIdle(void);
void displayErrorCode(uint8_t _code);
void displayWarningCode(uint8_t _code);
void buttonIndicator(uint8_t _value);
void menuIndicator(uint8_t _value);
#ifdef __cplusplus
}
#endif

#endif /* __SEVEN_SEGMENT_H */
