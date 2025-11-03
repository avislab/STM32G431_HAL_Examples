/*
 * ws2812.c
 *
 *  Created on: Oct 27, 2025
 *      Author: andre
 */

/*
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	WS2812_SendFinishedCallback(&ws2812);
}
*/

#include "main.h"
#include "ws2812.h"
#include "math.h"

void WS2812_Init(WS2812Struct *hWS2812, TIM_HandleTypeDef *htim, uint32_t Channel) {
	hWS2812->htim = htim;
	hWS2812->Channel = Channel;
	hWS2812->SendingFlag = 0;
	hWS2812->Brightness = 45;
	hWS2812->BrightnessK = 1.0f;

	hWS2812->pwm0 = (htim->Init.Period+1) / 3;
	hWS2812->pwm1 = ((htim->Init.Period+1) * 2) / 3;
}


void WS2812_SetBrightness(WS2812Struct *hWS2812, uint8_t Brightness) {
	hWS2812->Brightness = Brightness;

	Brightness = ((uint16_t)Brightness * 45) / 100;

	if (Brightness > 45) {
		Brightness = 45;
	}

	float angle = 90-hWS2812->Brightness; // degrees
	angle = angle*M_PI / 180;  // rad
	hWS2812->BrightnessK = tanf(angle);
}


void WS2812_SetLED (WS2812Struct *hWS2812, uint8_t LED, uint8_t Red, uint8_t Green, uint8_t Blue) {
	hWS2812->LEDData[LED][0] = Green;
	hWS2812->LEDData[LED][1] = Red;
	hWS2812->LEDData[LED][2] = Blue;
}

void WS2812_Send(WS2812Struct *hWS2812) {
	uint32_t indx=0;
	uint32_t color;
	uint8_t Green, Red, Blue;

	// Waiting for finish of previous sending
	while (hWS2812->SendingFlag != 0)
	{
		__NOP();
	}
	hWS2812->SendingFlag = 1;

	// Make a PWM Data
	for (uint8_t i= 0; i < WS2812_LEDS; i++)
	{
		Green = hWS2812->LEDData[i][0]/hWS2812->BrightnessK;
		Red = hWS2812->LEDData[i][1]/hWS2812->BrightnessK;
		Blue = hWS2812->LEDData[i][2]/hWS2812->BrightnessK;

		color = ((Green<<16) | (Red<<8) | (Blue));
		for (int8_t i=23; i>=0; i--)
		{
			if ((color&(1<<i)) != 0x0)
			{
				hWS2812->PWMData[indx] = hWS2812->pwm1;
			} else {
				hWS2812->PWMData[indx] = hWS2812->pwm0;
			}
			indx++;
		}
	}

	// Add Pause
	for (int i=0; i<50; i++)
	{
		hWS2812->PWMData[indx] = 0;
		indx++;
	}

	// Start Sending
	HAL_TIM_PWM_Start_DMA(hWS2812->htim, hWS2812->Channel, (uint32_t *)hWS2812->PWMData, indx);
}


void WS2812_SendFinishedCallback(WS2812Struct *hWS2812) {
	HAL_TIM_PWM_Stop_DMA(hWS2812->htim, hWS2812->Channel);
	hWS2812->SendingFlag = 0;
}
