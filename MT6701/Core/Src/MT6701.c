/*
 * MT6701.c
 *
 *  Created on: 30.05.2026.
 *      Author: andrey
 */

#include "main.h"
#include "MT6701.h"

void MT6701_Init(MT6701TypeDef *MT6701, I2C_HandleTypeDef *hi2c, uint8_t address)
{
	MT6701->hi2c = hi2c;
	MT6701->address = address;
}


uint8_t MT6701_nanbnz_enable(MT6701TypeDef *MT6701, uint8_t nanbnz_enable)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_UVM_MUX, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (nanbnz_enable == 1) {
		data |= MT6701_REG_UVM_MUX_MASK;
	} else {
		data &= ~MT6701_REG_UVM_MUX_MASK;
	}

	//if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_UVM_MUX, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
	//	return MT6701_ERR_IO;
	//}

	return MT6701_OK;
}


uint8_t MT6701_abz_pulse_per_round_set(MT6701TypeDef *MT6701, uint16_t resolution)
{
	uint8_t data;

	resolution--;
	if(resolution >= 1024){
		return MT6701_ERR_OUT_OF_RANGE;
	}

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ABZ_RES8, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	resolution >>= 8;
	data &= ~MT6701_REG_ABZ_RES8_MASK;
	data |= (uint8_t)(resolution << MT6701_REG_ZERO8_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ABZ_RES8, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_uvw_pole_pair_set(MT6701TypeDef *MT6701, uint8_t pole_pairs)
{
	uint8_t data;

	pole_pairs--;
	if(pole_pairs >= 16) {
		return MT6701_ERR_OUT_OF_RANGE;
	}

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_UVW_RES0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	data &= ~MT6701_REG_UVW_RES0_MASK;
	data |= (pole_pairs << MT6701_REG_UVW_RES0_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_UVW_RES0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_mode_set(MT6701TypeDef *MT6701, mt6701_mode_t mode)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ABZ_MUX, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (mode == MT6701_MODE_UVW)
	{
		data |=  MT6701_REG_ABZ_MUX_MASK;
	} else if(mode == MT6701_MODE_ABZ) {
		data &= ~MT6701_REG_ABZ_MUX_MASK;
	}

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ABZ_MUX, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_zero_set_raw(MT6701TypeDef *MT6701, uint16_t zero_angle)
{
	uint8_t data;

	zero_angle &= 0x3FFF;

	data = (uint8_t)(zero_angle & 0xFF);
	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ZERO0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ZERO8, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	zero_angle >>= 8;
	data &= ~MT6701_REG_ZERO8_MASK;
	data |= (zero_angle << MT6701_REG_ZERO8_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ZERO8, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_zero_set(MT6701TypeDef *MT6701, float zero_angle)
{
	uint16_t data;
	data = (uint16_t)(zero_angle * (16384.0f/360.0f));
	return MT6701_zero_set_raw(MT6701, data);
}

uint8_t MT6701_hyst_set(MT6701TypeDef *MT6701, mt6701_hyst_t hysteresis)
{
	uint8_t hyst_lo;
	uint8_t hyst_hi;
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_HYST0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	hyst_lo = hysteresis & 0x03;
	data &= ~MT6701_REG_HYST0_MASK;
	data |= (hyst_lo << MT6701_REG_HYST0_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_HYST0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_HYST2, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	hyst_hi = hysteresis >> 2;
	data &= ~MT6701_REG_HYST2_MASK;
	data |= (hyst_hi << MT6701_REG_HYST2_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_HYST2, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_a_start_stop_set_raw(MT6701TypeDef *MT6701, uint16_t start, uint16_t stop)
{
	uint8_t data;

	if(start >= 4096){
		return MT6701_ERR_OUT_OF_RANGE;
	}

	if(stop >= 4096){
		return MT6701_ERR_OUT_OF_RANGE;
	}

	data = (uint8_t)(start & 0xFF);
	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_A_START0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	data = (uint8_t)(stop & 0xFF);
	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_A_STOP0, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	start >>= 8;
	stop  >>= 8;
	data = (start << MT6701_REG_A_START8_POS) | (stop << MT6701_REG_A_STOP8_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_A_START8, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_a_start_stop_set(MT6701TypeDef *MT6701, float start, float stop)
{
	uint16_t start_u16;
	uint16_t stop_u16;

	start_u16 = (uint16_t)(start * (4096.0f/360.0f));
	stop_u16  = (uint16_t)(stop * (4096.0f/360.0f));
	if(start_u16 >= 4096){
		start_u16 = 0;
	}
	if(stop_u16 >= 4096){
		stop_u16 = 4095;
	}
	return MT6701_a_start_stop_set_raw(MT6701, start_u16, stop_u16);
}

uint8_t mt6701_direction_set(MT6701TypeDef *MT6701, mt6701_direction_t direction)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_DIR, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (direction == MT6701_DIRECTION_CW)
	{
		data &= ~MT6701_REG_DIR_MASK;
	} else if (direction == MT6701_DIRECTION_CCW){
		data |=  MT6701_REG_DIR_MASK;
	} else {
		return MT6701_ERR_GENERAL;
	}

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_DIR, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	return MT6701_OK;
}

uint8_t MT6701_pulse_width_set(MT6701TypeDef *MT6701, mt6701_pulse_width_t pulse_width)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PULSE_WIDTH, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	data &= ~MT6701_REG_PULSE_WIDTH_MASK;
	data |= (pulse_width << MT6701_REG_PULSE_WIDTH_POS);

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PULSE_WIDTH, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}
	return MT6701_OK;
}

uint8_t mt6701_pwm_freq_set(MT6701TypeDef *MT6701, mt6701_pwm_freq_t pwm_freq)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PWM_FREQ, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (pwm_freq == MT6701_PWM_FREQ_994_4)
	{
		data &= ~MT6701_REG_PWM_FREQ_MASK;
	} else if (pwm_freq == MT6701_PWM_FREQ_497_2){
		data |=  MT6701_REG_PWM_FREQ_MASK;
	} else {
		return MT6701_ERR_GENERAL;
	}

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PWM_FREQ, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}
	return MT6701_OK;
}

uint8_t MT6701_pwm_polarity_set(MT6701TypeDef *MT6701, mt6701_pwm_pol_t pwm_polarity)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PWM_POL, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (pwm_polarity == MT6701_PWM_POL_HIGH)
	{
		data &= ~MT6701_REG_PWM_POL_MASK;
	} else if (pwm_polarity == MT6701_PWM_POL_LOW) {
		data |=  MT6701_REG_PWM_POL_MASK;
	} else {
		return MT6701_ERR_GENERAL;
	}

	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PWM_POL, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}
	return MT6701_OK;
}

uint8_t MT6701_out_mode_set(MT6701TypeDef *MT6701, mt6701_out_mode_t out_mode)
{
	uint8_t data;

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_PWM_POL, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (out_mode == MT6701_OUT_MODE_ANALOG)
	{
		data &= ~MT6701_REG_OUT_MODE_MASK;
	} else if (out_mode == MT6701_OUT_MODE_PWM) {
		data |=  MT6701_REG_OUT_MODE_MASK;
	} else {
		return MT6701_ERR_GENERAL;
	}

	//if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, MT6701_REG_OUT_MODE, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
	//	return MT6701_ERR_IO;
	//}
	HAL_I2C_Master_Transmit(MT6701->hi2c, MT6701->address, &data, 1, 100);

	return MT6701_OK;
}

uint8_t MT6701_programm_eeprom(MT6701TypeDef *MT6701)
{
	uint8_t data;

	data = 0xB3;
	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, 0x09, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	data = 0x05;
	if (HAL_I2C_Mem_Write(MT6701->hi2c, MT6701->address << 1, 0x0A, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	HAL_Delay(650);
	return MT6701_OK;
}

uint8_t MT6701_read_raw(MT6701TypeDef *MT6701, uint16_t *angle_raw)
{
	uint16_t angle_u16;
	uint8_t data[2];

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ANGLE6, 1, &data[1], 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	if (HAL_I2C_Mem_Read(MT6701->hi2c, MT6701->address << 1, MT6701_REG_ANGLE0, 1, &data[0], 1, HAL_MAX_DELAY) != HAL_OK) {
		return MT6701_ERR_IO;
	}

	angle_u16  = (uint16_t)(data[0] >> MT6701_REG_ANGLE0_POS);
	angle_u16 |= ((uint16_t)data[1] << (8-MT6701_REG_ANGLE0_POS));

	if(angle_raw != NULL){
		*angle_raw = angle_u16;
	}
	return MT6701_OK;
}

uint8_t MT6701_read(MT6701TypeDef *MT6701, float *angle)
{
	uint8_t res;
	uint16_t angle_u16;
	float angle_f;

	res = MT6701_read_raw(MT6701, &angle_u16);
	if (res != 0) {
		return res;
	}

	if (angle != NULL) {
		angle_f = (float)angle_u16 * (360.0f/16384.0f);
		*angle = angle_f;
	}

	return MT6701_OK;
}
