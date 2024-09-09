#include "seven_segment.h"

// Array of segments
uint16_t ALL_SEGMENTS[7] = {SEG_A_Pin, SEG_B_Pin, SEG_C_Pin, SEG_D_TEMPORARY_Pin, SEG_E_Pin, SEG_F_Pin, SEG_G_Pin};

uint8_t convertNumber(uint8_t num)
{
	uint8_t converted;
	switch (num)
	{
		case (0x00):
			converted = NOL;
			break;
		case (0x01):
			converted = SATU;
			break;
		case (0x02):
			converted = DUA;
			break;
		case (0x03):
			converted = TIGA;
			break;
		case (0x04):
			converted = EMPAT;
			break;
		case (0x05):
			converted = LIMA;
			break;
		case (0x06):
			converted = ENAM;
			break;
		case (0x07):
			converted = TUJUH;
			break;
		case (0x08):
			converted = DELAPAN;
			break;
		case (0x09):
			converted = SEMBILAN;
			break;
		case (0x45):
			converted = CHAR_E;
			break;
		case (0x52):
			converted = CHAR_R;
			break;
	}
	return converted;
}

void checkSegment(void)
{
	GPIO_TypeDef* _PORT;
	
	COM1_ON;
	COM2_ON;
	COM3_ON;
	COM4_ON;
	for(uint8_t i=0; i<7; i++){
		if(i == 0)
			_PORT = GPIOB;
		else
			_PORT = GPIOA;
		HAL_GPIO_WritePin(_PORT, ALL_SEGMENTS[i], GPIO_PIN_SET);
	}
}