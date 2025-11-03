/*
 * sound.c
 *
 *  Created on: 17 окт. 2025 г.
 *      Author: andrey
 */

#include "main.h"
#include "sound.h"

extern TIM_HandleTypeDef htim3;

int MusicStep = 0;
char PlayMusic = 0;

int sound_time;
int sound_counter;

#define MUSICSIZE 48
const SoundTypeDef Music[MUSICSIZE] ={
	{C*2, t4},
	{G, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C, t2},
	{C*2, t4},
	{G, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C*2, t4},
	{0, t8},
	{D_, t8},
	{D_, t8},
	{D_, t8},
	{G, t8},
	{A_, t4},
	{D_*2, t8},
	{C_*2, t8},
	{C*2, t8},
	{C*2, t8},
	{C*2, t8},
	{C*2, t8},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C*2, t2},
	{C*2, t2},
	{A_, t8},
	{G_, t8},
	{G, t8},
	{G_, t8},
	{A_, t2},
	{A_, t4},
	{C*2, t4},
	{A_, t8},
	{F, t8},
	{D_, t8},
	{F, t8},
	{G, t4},
	{C*2, t2}
};

void sound (int freq, int time_ms) {
	if (freq > 0) {
		TIM3->ARR = 1000000 / freq;
		TIM3->CCR2 = TIM3->ARR / 2;
	}
	else {
		TIM3->ARR = 1000;
		TIM3->CCR2 = 0;
	}
	TIM3->CNT = 0;

	sound_time = ((1000000 / TIM3->ARR) * time_ms ) / 1000;
	sound_counter = 0;

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void StartMusic(void) {
	MusicStep = 0;
	PlayMusic = 1;
	sound(Music[MusicStep].freq, Music[MusicStep].time);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    sound_counter++;
    if (sound_counter > sound_time) {
    	if (PlayMusic == 0) {
    		HAL_TIM_Base_Stop(&htim3);
    		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    	}
    	else {
    		if (MusicStep < MUSICSIZE-1) {
    			if (TIM3->CCR1 == 0){
    				MusicStep++;
    				sound(Music[MusicStep].freq, Music[MusicStep].time);
    			}
    			else{
    				sound(0, 30);
    			}
    		}
    		else {
	    		PlayMusic = 0;
	    		HAL_TIM_Base_Stop(&htim3);
	    		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    		}
    	}
    }
}

