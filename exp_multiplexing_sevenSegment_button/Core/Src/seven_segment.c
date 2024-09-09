#include "seven_segment.h"

// Array of segments and COMs Pin Number
uint16_t allSegments[8] = {SEG_A_Pin, SEG_B_Pin, SEG_C_Pin, SEG_D_Pin, SEG_E_Pin, SEG_F_Pin, SEG_G_Pin, SEG_DOT_Pin};

typedef struct struct_displayTick
{
	_Bool	blinkState;
	uint8_t count;
	uint32_t lastTime;
}struct_displayTick;

switching_delay_t blinkDot 	= {0, false};
struct_displayTick loadingAnim	= {false, 0, 0};

/**
	*	Convert decimal number to seven segments display
	* @parameter num:	decimal number (uint8_t)
**/
uint8_t convertValue(uint8_t num)
{
	uint8_t converted = 0x00;
	switch (num)
	{
		case (0x00):
		case (0x6F):
			converted = NOL;
			break;
		case (0x01):
		case (0x6C):
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
		case (0x61):
			converted = CHAR_a;
			break;
		case (0x43):
			converted = CHAR_C;
			break;
		case (0x64):
			converted = CHAR_d;
			break;
		case (0x45):
			converted = CHAR_E;
			break;
		case (0x65):
			converted = CHAR_e;
			break;
		case (0x68):
			converted = CHAR_h;
			break;
		case (0x6E):
			converted = CHAR_n;
			break;
		case (0x50):
			converted = CHAR_P;
			break;
		case (0x52):
			converted = CHAR_R;
			break;
	}
	return converted;
}

/*	Check segments and COM	*/
void checkSegment(void)
{
	COM1_ON;
	COM2_ON;
	COM3_ON;
	COM4_ON;
	DOUBLE_DOT_ON;
	for(uint8_t i=0; i<7; i++){
		HAL_GPIO_WritePin(GPIOA, allSegments[i], GPIO_PIN_SET);
	}
}

/**
	* Write data to segments
	* @parameter	value:	converted number from convertValue function
* @parameter	_useDot:	Dot value
**/
void writeSegment(uint8_t value, _Bool _useDot)
{
	if(_useDot == true)
		value = value + 0x80;			// Double dot

	for(uint8_t i=0; i<8; i++){
		HAL_GPIO_WritePin(GPIOA, allSegments[i], (value&0x01));
		value = value >> 1;
	}
}


/**
	*	Display desired value on COM1
	*	@parameter _value:	Value to display
**/
void digitCOM1(uint8_t _value)
{
	blankDisplay();
	writeSegment(_value, false);
	COM1_ON;
	delay_us(COM_SWITCH_DELAY);
}

/**
	*	Display desired value on COM2
	*	@parameter _value:	Value to display
**/
void digitCOM2(uint8_t _value, _Bool _useDot)
{
	blankDisplay();
	writeSegment(_value, _useDot);
	COM2_ON;
	delay_us(COM_SWITCH_DELAY);
}

/**
	*	Display desired value on COM3
	*	@parameter _value:	Value to display
**/
void digitCOM3(uint8_t _value, _Bool _useDot)
{
	blankDisplay();
	writeSegment(_value, _useDot);
	COM3_ON;
	delay_us(COM_SWITCH_DELAY);
}

/**
	*	Display desired value on COM4
	*	@parameter _value:	Value to display
**/
void digitCOM4(uint8_t _value)
{
	blankDisplay();
	writeSegment(_value, false);
	COM4_ON;
	delay_us(COM_SWITCH_DELAY);
}

/**
	*	Display desired value on COM4
	*	@parameter _value:	Value to display
**/
void digitCOM5(uint8_t _value)
{
	blankDisplay();
	writeSegment(_value, false);
	COM5_ON;
	delay_us(COM_SWITCH_DELAY);
}

/**
	*	Display desired value on COM4
	*	@parameter _value:	Value to display
**/
void digitCOM6(uint8_t _value)
{
	blankDisplay();
	writeSegment(_value, false);
	COM6_ON;
	delay_us(COM_SWITCH_DELAY);
}

/**
	*	Blank display by turning off the all COM
**/
void blankDisplay(void)
{
	COM1_OFF;
	COM2_OFF;
	COM3_OFF;
	COM4_OFF;
	COM5_OFF;
	COM6_OFF;
}
/**
	*	Display desired value to segment
	*	@parameter value:	Desired decimal value
**/
void displayTimer(uint32_t inputSecond, _Bool blinkAll, _Bool hoursMinutesOnly)
{
	// Reset the animation counter if rice or red rice mode selected
	loadingAnim.count = 0;

	// Digit Value
	uint8_t tensOfHourDigit, hourDigit, tensOfMinuteDigit, minuteDigit, tensOfSecondDigit, secondDigit;

	// Time Values
	uint16_t _hours	 = (inputSecond / 3600);
	uint16_t _minutes = (inputSecond - (3600 * _hours))/60;
	uint16_t _seconds = (inputSecond - (3600 * _hours) - (_minutes * 60));

	// Calculate values
	uint16_t tensOfHourValue	= (_hours / 10) % 10;
	uint16_t hourValue			= _hours % 10;
	uint16_t tensOfMinuteValue 	= (_minutes / 10) % 10;
	uint16_t minuteValue 		= _minutes % 10;
	uint16_t tensOfSecondValue	= (_seconds / 10) %10;
	uint16_t secondValue		= _seconds % 10;

	// Hours Value
	if (_hours > 0 && _hours < MAX_HOURS)
	{
		tensOfHourDigit = convertValue(tensOfHourValue);
		hourDigit = convertValue(hourValue);
	}
	else
	{
		tensOfHourDigit = convertValue(0x00);
		hourDigit = convertValue(0x00);
	}

	// Minutes Value
	if (_minutes >= 0 && _minutes < 60)
	{
		tensOfMinuteDigit = convertValue(tensOfMinuteValue);
		minuteDigit = convertValue(minuteValue);
	}
	else
	{
		tensOfMinuteDigit = convertValue(0x00);
		minuteDigit = convertValue(0x00);
	}

	// Seconds Value
	if (_seconds >= 0 && _seconds < 60)
	{
		tensOfSecondDigit = convertValue(tensOfSecondValue);
		secondDigit = convertValue(secondValue);
	}
	else
	{
		tensOfSecondDigit = convertValue(0x00);
		secondDigit = convertValue(0x00);
	}

	// Control Dot
	intervalSwitching(&blinkDot, BLINK_DELAY);

	// Display conditional
	if(_hours == 0 && _minutes == 0 && _seconds >= 0 && !hoursMinutesOnly)
	{
		digitCOM4(secondDigit);										// Seconds
		if(_seconds >= 10)
			digitCOM3(tensOfSecondDigit, false);		// Tens of Second
	}
	else
	{
		if (tensOfHourValue != 0)
			digitCOM1(tensOfHourDigit);						// Tens of Hour

		// Blinking the dot
		blankDisplay();
		if(blinkDot._state == true || blinkAll == false)
		{
			digitCOM2(hourDigit, true);						// Hours
			digitCOM3(tensOfMinuteDigit, true);				// Tens of Minute
		}
		else
		{
			digitCOM2(hourDigit, false);					// Hours
			digitCOM3(tensOfMinuteDigit, false);			// Tens of Minute
		}
		digitCOM4(minuteDigit);								// Minutes
	}
}

/**
	*	Display desired text to display
	*	@parameter _text:	Pointer to ASCII character
**/
void displayText(const uint8_t* _text, uint32_t _delay)
{
	blankDisplay();
	for(uint32_t i=0; i<_delay; i++)
	{
		digitCOM1(convertValue(_text[0]));
		digitCOM2(convertValue(_text[1]), false);
		digitCOM3(convertValue(_text[2]), false);
		digitCOM4(convertValue(_text[3]));
	}
}

/**
	*	Display mode with CODE
	*	@parameter _mode:	Pointer to ASCII character
**/
void displayMode(uint8_t _mode)
{
	blankDisplay();
	digitCOM2(convertValue(0x43), false);
	digitCOM3(convertValue(0x00), false);
	digitCOM4(convertValue(_mode));
}

/**
	*	Display loading animation while cooking rice
* @param : delay for displaying the characters
**/
void displayLoading()
{
	blankDisplay();
	if(HAL_GetTick()- loadingAnim.lastTime > 100)
	{
		loadingAnim.lastTime = HAL_GetTick();
		loadingAnim.count += 1;

		// Reset the loading counter
		if(loadingAnim.count > 11)
			loadingAnim.count = 0;
	}

	switch(loadingAnim.count)
	{
		case 0:
			digitCOM1(0x10);
			digitCOM2(0x00, false);
			digitCOM3(0x00, false);
			digitCOM4(0x00);
			break;
		case 1:
			digitCOM1(0x30);
			digitCOM2(0x00, false);
			digitCOM3(0x00, false);
			digitCOM4(0x00);
			break;
		case 2:
			digitCOM1(0x31);
			digitCOM2(0x00, false);
			digitCOM3(0x00, false);
			digitCOM4(0x00);
			break;
		case 3:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x00, false);
			digitCOM4(0x00);
			break;
		case 4:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x01, false);
			digitCOM4(0x00);
			break;
		case 5:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x01, false);
			digitCOM4(0x01);
			break;
		case 6:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x01, false);
			digitCOM4(0x03);
			break;
		case 7:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x01, false);
			digitCOM4(0x07);
			break;
		case 8:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x01, false);
			digitCOM4(0x0F);
			break;
		case 9:
			digitCOM1(0x31);
			digitCOM2(0x01, false);
			digitCOM3(0x09, false);
			digitCOM4(0x0F);
			break;
		case 10:
			digitCOM1(0x31);
			digitCOM2(0x09, false);
			digitCOM3(0x09, false);
			digitCOM4(0x0F);
			break;
		case 11:
			digitCOM1(0x08);
			digitCOM2(0x09, false);
			digitCOM3(0x09, false);
			digitCOM4(0x0F);
			break;
	}
}

/**
	*	Display idle condition/no menu selected
**/
void displayIdle(void)
{
	digitCOM1(0x40);
	digitCOM2(0x40, false);
	digitCOM3(0x40, false);
	digitCOM4(0x40);

}

/**
*	@brief Display system error code
*	@param _code Error Code
**/
void displayErrorCode(uint8_t _code)
{
	digitCOM2(convertValue(0x45), false);
	digitCOM3(convertValue(0x00), false);
	digitCOM4(convertValue(_code));
}

/**
*	@brief Display system warning code
*	@param _code Error Code
**/
void displayWarningCode(uint8_t _code)
{
	for(uint32_t i=0; i<300; i++)
	{
		digitCOM2(convertValue(0x50), false);
		digitCOM3(convertValue(0x00), false);
		digitCOM4(convertValue(_code));
	}
}

/**
 *	Menu Indicator
**/
void menuIndicator(uint8_t _value)
{
	blankDisplay();
	digitCOM5(_value);
}

/**
 *	Button Indicator
**/
void buttonIndicator(uint8_t _value)
{
	blankDisplay();
	digitCOM6(_value);
}
