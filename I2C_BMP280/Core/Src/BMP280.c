/*
 * BMP280.c
 *
 *  Created on: 13 окт. 2025 г.
 *      Author: andrey
 */

#include "main.h"
#include "BMP280.h"

short bmp280ReadByte(BMP280TypeDef *BMP280, uint16_t address)
{
	if (HAL_I2C_Mem_Read(BMP280->hi2c, BMP280->address << 1, address, 1, &BMP280->buf[0], 1, HAL_MAX_DELAY) != HAL_OK) {
	  Error_Handler();
	}

	return BMP280->buf[0];
}

uint16_t bmp280ReadShort(BMP280TypeDef *BMP280, uint16_t address)
{
	if (HAL_I2C_Mem_Read(BMP280->hi2c, BMP280->address << 1, address, 1, &BMP280->buf[0], 2, HAL_MAX_DELAY) != HAL_OK) {
	  Error_Handler();
	}

	return ((uint16_t)BMP280->buf[1] << 8) | BMP280->buf[0];
}

uint32_t bmp280ReadLong(BMP280TypeDef *BMP280, uint16_t address)
{
	if (HAL_I2C_Mem_Read(BMP280->hi2c, BMP280->address << 1, address, 1, &BMP280->buf[0], 3, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}

	return ((BMP280->buf[0] << 16) | (BMP280->buf[1] << 8) | BMP280->buf[2]) >> 4;
}

void bmp280WriteByte(BMP280TypeDef *BMP280, uint16_t address, uint8_t data)
{
	BMP280->buf[0] = data;
	if (HAL_I2C_Mem_Write(BMP280->hi2c, BMP280->address << 1, address, 1, &BMP280->buf[0], 1, HAL_MAX_DELAY) != HAL_OK) {
	        Error_Handler();
	}
}

void BMP280_Init(BMP280TypeDef *BMP280, I2C_HandleTypeDef *hi2c, uint8_t address) {
	BMP280->hi2c = hi2c;
	BMP280->address = address;

	BMP280->id = bmp280ReadByte(BMP280, BMP280_REG_ID);


	// Soft reset
	bmp280WriteByte(BMP280, BMP280_REG_RESET, BMP280_RESET_VALUE);

	// Wait until finished copying over the NVP data
	while (1) {
		if (bmp280ReadByte(BMP280, BMP280_REG_STATUS) == 0) {
			break;
		}
	}

	// Read calibration data
	BMP280->dig_T1 = (uint16_t)bmp280ReadShort(BMP280, 0x88);//dig_T1
	BMP280->dig_T2 = bmp280ReadShort(BMP280, 0x8A);//dig_T2
	BMP280->dig_T3 = bmp280ReadShort(BMP280, 0x8C);//dig_T3
	BMP280->dig_P1 = (uint16_t)bmp280ReadShort(BMP280, 0x8E);//dig_P1
	BMP280->dig_P2 = bmp280ReadShort(BMP280, 0x90);//dig_P2
	BMP280->dig_P3 = bmp280ReadShort(BMP280, 0x92);//dig_P3
	BMP280->dig_P4 = bmp280ReadShort(BMP280, 0x94);//dig_P4
	BMP280->dig_P5 = bmp280ReadShort(BMP280, 0x96);//dig_P5
	BMP280->dig_P6 = bmp280ReadShort(BMP280, 0x98);//dig_P6
	BMP280->dig_P7 = bmp280ReadShort(BMP280, 0x9A);//dig_P7
	BMP280->dig_P8 = bmp280ReadShort(BMP280, 0x9C);//dig_P8
	BMP280->dig_P9 = bmp280ReadShort(BMP280, 0x9E);//dig_P9

	bmp280WriteByte(BMP280, BMP280_REG_CONFIG, BMP280_CONFIG);
	bmp280WriteByte(BMP280, BMP280_REG_CONTROL, BMP280_MEAS);
}

void BMP280_Convert(BMP280TypeDef *BMP280, int32_t* temperature, int32_t* pressure) {
	int32_t adc_temp;
	int32_t adc_pressure;
	adc_temp = bmp280ReadLong(BMP280, BMP280_REG_RESULT_TEMPRERATURE);
	adc_pressure = bmp280ReadLong(BMP280, BMP280_REG_RESULT_PRESSURE);

	int32_t fine_temp;
	int64_t var1, var2, p;

	// Temperature
	var1 = ((((adc_temp >> 3) - ((int32_t) BMP280->dig_T1 << 1))) * (int32_t) BMP280->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) BMP280->dig_T1) * ((adc_temp >> 4) - (int32_t) BMP280->dig_T1)) >> 12) * (int32_t) BMP280->dig_T3) >> 14;
	fine_temp = var1 + var2;
	*temperature = (fine_temp * 5 + 128) >> 8;


	// Pressure
	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) BMP280->dig_P6;
	var2 = var2 + ((var1 * (int64_t) BMP280->dig_P5) << 17);
	var2 = var2 + (((int64_t) BMP280->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) BMP280->dig_P3) >> 8) + ((var1 * (int64_t) BMP280->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) BMP280->dig_P1) >> 33;

	if (var1 == 0) {
		return;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_pressure;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) BMP280->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) BMP280->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) BMP280->dig_P7 << 4);
	*pressure = p*100 / 256;
}
