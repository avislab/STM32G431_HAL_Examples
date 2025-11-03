/*
 * user.h
 *
 *  Created on: Sep 25, 2025
 *      Author: andre
 */

#ifndef INC_USER_H_
#define INC_USER_H_

#include "main.h"

#define KEY_IS_PRESSED	HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET
#define LED_ON	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_OFF	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_BLINK_NUMBER	5

void USER_loop(void);

#endif /* INC_USER_H_ */
