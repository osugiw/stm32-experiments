/*
 * led.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: KDS-LTP-0415
 */

#include "led.h"

//led::led() {
//	// TODO Auto-generated constructor stub
//
//}
//
//led::~led() {
//	// TODO Auto-generated destructor stub
//}

void led_main::on(){
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
}

void led_main::off(){
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
}
