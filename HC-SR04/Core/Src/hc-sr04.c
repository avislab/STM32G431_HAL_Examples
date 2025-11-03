/*
 * hc-sr04.c
 *
 *  Created on: Oct 23, 2025
 *      Author: andre
 */

#include "main.h"
#include "dwt.h"
#include "hc-sr04.h"

extern TIM_HandleTypeDef htim6;

volatile uint16_t SonarTIMValue = 0;
volatile uint8_t FLAG_ECHO = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
     if (GPIO_Pin == ECHO_Pin) {
    	 if(HAL_GPIO_ReadPin(TRIG_GPIO_Port, TRIG_Pin) == GPIO_PIN_SET) {
    		 __HAL_TIM_SET_COUNTER(&htim6, 0);
    	 } else {
    		 SonarTIMValue = __HAL_TIM_GET_COUNTER(&htim6);
    	 }
     }
}

void SonarStartMeasuring() {
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	DWTDelay_us(15);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	htim6.Instance->CNT = 0;
}

unsigned int SonarGetDistance() {
	unsigned long Sonar;
	// 354000 - Sound speed (mm/sec)
	// 170000000 - F_CPU
	// 16 - Timer Prescaler
	// Result = mm

	Sonar = (354/2) * (unsigned long)SonarTIMValue / (170000 / 170);
	if (Sonar > 4000) Sonar = 4000;
	if (Sonar < 20) Sonar = 20;

	return (unsigned int)Sonar;
}
