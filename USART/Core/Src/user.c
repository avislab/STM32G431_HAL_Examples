/*
 * user.c
 *
 *  Created on: Sep 25, 2025
 *      Author: andre
 */
#include "user.h"

void USER_loop(void) {
	uint8_t i;

	if (KEY_IS_PRESSED) {
	  for (i=0; i<LED_BLINK_NUMBER; i++) {
		  LED_ON;
		  HAL_Delay(200);
		  LED_OFF;
		  HAL_Delay(200);
	  }

	  while (KEY_IS_PRESSED) {
		  HAL_Delay(100);
	  }
	}

	HAL_Delay(100);
}
