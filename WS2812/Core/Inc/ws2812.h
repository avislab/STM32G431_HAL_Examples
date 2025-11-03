/*
 * ws2812.h
 *
 *  Created on: Oct 27, 2025
 *      Author: andre
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "main.h"

#define WS2812_LEDS 8

typedef struct
{
	TIM_HandleTypeDef *htim;
	uint32_t Channel;

	uint8_t Brightness;
	float BrightnessK;

	uint8_t pwm0;
	uint8_t pwm1;

	uint8_t SendingFlag;
	uint8_t LEDData[WS2812_LEDS][3];
	uint16_t PWMData[24*WS2812_LEDS + 50];

} WS2812Struct;


void WS2812_Init(WS2812Struct *hWS2812, TIM_HandleTypeDef *htim, uint32_t Channel);
void WS2812_SetBrightness(WS2812Struct *hWS2812, uint8_t Brightness);
void WS2812_SetLED(WS2812Struct *hWS2812, uint8_t LED, uint8_t Red, uint8_t Green, uint8_t Blue);
void WS2812_Send(WS2812Struct *hWS2812);
void WS2812_SendFinishedCallback(WS2812Struct *hWS2812);

#endif /* INC_WS2812_H_ */
