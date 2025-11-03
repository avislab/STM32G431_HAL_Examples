/*
 * user.c
 *
 *  Created on: Sep 25, 2025
 *      Author: andre
 */
#include "user.h"

uint8_t KEY_Flag = 0;

void USER_loop(void) {
	uint8_t i;

	if (KEY_Flag == 1) {
	  for (i=0; i<LED_BLINK_NUMBER; i++) {
		  LED_ON;
		  HAL_Delay(200);
		  LED_OFF;
		  HAL_Delay(200);
	  }

	  KEY_Flag = 0;
	}

	HAL_Delay(100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	KEY_Flag = 1;
}
