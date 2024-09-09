/*
 * led.h
 *
 *  Created on: Oct 28, 2022
 *      Author: KDS-LTP-0415
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA

class led_main {
public:
//	led();
//	virtual ~led();
	void on();
	void off();
};

#ifdef __cplusplus
}
#endif
#endif /* SRC_LED_H_ */
