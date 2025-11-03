/*
 * MS5611.c
 *
 *  Created on: 12 окт. 2025 г.
 *      Author: andrey
 */

#include "main.h"
#include "MS5611.h"

short ms5611ReadShort(MS5611TypeDef *MS5611, unsigned char address)
{
	if (HAL_I2C_Mem_Read(MS5611->hi2c, MS5611->address << 1, address, 1, &MS5611->buf[0], 2, HAL_MAX_DELAY) != HAL_OK) {
	  Error_Handler();
	}

	return (MS5611->buf[0] << 8) | MS5611->buf[1];
}

unsigned long ms5611ReadLong(MS5611TypeDef *MS5611, unsigned char address)
{
	if (HAL_I2C_Mem_Read(MS5611->hi2c, MS5611->address << 1, address, 1, &MS5611->buf[0], 3, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}

	return (MS5611->buf[0] << 16) | (MS5611->buf[1] << 8) | MS5611->buf[2];
}

void ms5611WriteByte(MS5611TypeDef *MS5611, unsigned char data)
{
	MS5611->buf[0] = data;
	if (HAL_I2C_Master_Transmit(MS5611->hi2c, MS5611->address << 1, &MS5611->buf[0], 1, HAL_MAX_DELAY) != HAL_OK) {
	        Error_Handler();
	}
}


void MS5611_Init(MS5611TypeDef *MS5611, I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t OSR) {
	MS5611->hi2c = hi2c;
	MS5611->address = address;
	MS5611->OSR = OSR;

	// Reset
	ms5611WriteByte(MS5611, 0x1E);

	HAL_Delay(2);

	// Read calibration data
	MS5611->C1 = ms5611ReadShort(MS5611, 0xA2);
	MS5611->C2 = ms5611ReadShort(MS5611, 0xA4);
	MS5611->C3 = ms5611ReadShort(MS5611, 0xA6);
	MS5611->C4 = ms5611ReadShort(MS5611, 0xA8);
	MS5611->C5 = ms5611ReadShort(MS5611, 0xAA);
	MS5611->C6 = ms5611ReadShort(MS5611, 0xAC);
}

void MS5611_Convert(MS5611TypeDef *MS5611, long* temperature, long* pressure) {

	unsigned long D1, D2;
	long dT, TEMP;
	long long OFF, SENS, OFF2, SENS2, T2;

	// Start Pressure conversion
	ms5611WriteByte(MS5611, 0x40 + MS5611->OSR * 2);
	HAL_Delay(1 + MS5611->OSR * 2);
	// Read Pressure data
	D1 = ms5611ReadLong(MS5611, 0x00);

	// Start Temperature conversion
	ms5611WriteByte(MS5611, 0x50 + MS5611->OSR * 2);
	HAL_Delay(1 + MS5611->OSR * 2);
	// Read Temperature data
	D2 = ms5611ReadLong(MS5611, 0x00);

	dT = D2 - (MS5611->C5 << 8);
	TEMP = 2000 + (((long long)dT * (long long)MS5611->C6) >> 23);
	OFF = ((long long)MS5611->C2 << 16) + (((long long)MS5611->C4 * (long long)dT) >> 7);
	SENS = ((long long)MS5611->C1 << 15 ) + (((long long)MS5611->C3 * (long long)dT ) >> 8);

	if (TEMP >= 2000) {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	else if (TEMP < 2000) {
		T2 = (((long long)dT * (long long)dT) >> 31);
		OFF2 = 5 * (((long long)TEMP - 2000) * ((long long)TEMP - 2000)) >> 1;
		SENS2 = 5 * (((long long)TEMP - 2000) * ((long long)TEMP - 2000)) >> 2;
		if (TEMP < -1500 ) {
			OFF2 = OFF2 + 7 * (((long long)TEMP + 1500) * ((long long)TEMP + 1500));
			SENS2 = SENS2 + ((11 *(((long long)TEMP + 1500) * ((long long)TEMP + 1500))) >> 1);
		}
	}

	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	*pressure = (unsigned long) (((((D1 * SENS) >> 21) - OFF)) >> 15);
	*temperature = (long)TEMP/10;
}
