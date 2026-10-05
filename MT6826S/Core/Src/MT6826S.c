/*
 * MT6826S.c
 *
 *  Created on: Jun 2, 2026
 *      Author: andre
 */

#include "main.h"
#include "MT6826S.h"

void MT6826S_Init(MT6826STypeDef *MT6826S, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CSNPort, uint16_t CSNPin)
{
	MT6826S->hspi = hspi;
	MT6826S->CSNPort = CSNPort;
	MT6826S->CSNPin = CSNPin;
	HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_SET);
}

uint8_t MT6826S_ReadRegister(MT6826STypeDef *MT6826S, uint16_t address)
{
    uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = ((uint8_t)MT6826S_READ_REG << 4) | ((address >> 8) & 0x0F);
    tx[1] = address & 0xFF;
    tx[2] = 0x00;

    HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(MT6826S->hspi, tx, rx, 3, 100);
    HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_SET);

    return rx[2];
}

HAL_StatusTypeDef MT6826S_WriteRegister(MT6826STypeDef *MT6826S, uint16_t address, uint8_t data)
{
    uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = ((uint8_t)MT6826S_WRITE_REG << 4) | ((address >> 8) & 0x0F);
    tx[1] = address & 0xFF;
    tx[2] = data;

    HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(MT6826S->hspi, tx, rx, 3, 100);
    HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef MT6826S_ProgramEEPROM(MT6826STypeDef *MT6826S)
{
	uint8_t i;
	uint8_t eeDone;
	uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = (MT6826S_EEPROM_PROG << 4);
    tx[1] = 0x00;
    tx[2] = 0x00;

    HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(MT6826S->hspi, tx, rx, 3, 100);
    HAL_GPIO_WritePin(MT6826S->CSNPort, MT6826S->CSNPin, GPIO_PIN_SET);

    if (rx[2] != MT6826S_SUCCESS_ACK) {
    	return HAL_ERROR;
    }

    /* Poll EE_DONE bit (register 0x112, bit 5) up to 20 times (10 s total). */
    for(i = 0; i < 25; i++){
    	eeDone = (MT6826S_ReadRegister(MT6826S, MT6826S_EE_DONE_REG) >> 5) & 0x01;
    	if (eeDone != 0) {
    		return HAL_OK;
    	}
    	HAL_Delay(500);
    }

	return HAL_ERROR;
}

HAL_StatusTypeDef MT6826S_ConfigZeroPosition(MT6826STypeDef *MT6826S, mt6826s_zero_pos_t ZERO_POS)
{
	/* Step 1: clear zero position registers to read absolute angle. */
	uint8_t abzReg4 = MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG4) & 0xF;
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG3, 0x00);
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG4, abzReg4);

	/* Step 2: wait for the sensor to settle, then read current angle. */
	HAL_Delay(120);
	uint16_t posAngle12bit = MT6826S_getRawAngle(MT6826S) >> 3;
	if(!ZERO_POS){
		posAngle12bit = 0;
	}

	/* Step 3: write the new zero position. */
	uint8_t abzReg3 = posAngle12bit >> 4;
	abzReg4 = abzReg4 | ((posAngle12bit & 0xF) << 4);

	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG3, abzReg3);
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG4, abzReg4);
	uint8_t check = (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG3) ^ abzReg3) | (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG4) ^ abzReg4);

	HAL_Delay(120);

	if (check == 0) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MT6826S_ConfigSpecificZeroPosition(MT6826STypeDef *MT6826S, uint16_t angle12Bit)
{
	uint8_t abzReg3 = (angle12Bit & 0xFF0) >> 4;
	uint8_t abzReg4 = (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG4) & 0xF) | ((angle12Bit & 0xF) << 4);
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG3, abzReg3);
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG4, abzReg4);
	uint8_t check = (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG3) ^ abzReg3) | (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG4) ^ abzReg4);

	HAL_Delay(120);

	if (check == 0) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MT6826S_ConfigDirection(MT6826STypeDef *MT6826S, mt6826s_rotation_direction_t ROTATION_DIRECTION)
{
	uint8_t rotRegister = (MT6826S_ReadRegister(MT6826S, MT6826S_ROT_REG) & 0b11110111) | (ROTATION_DIRECTION << 3);
	MT6826S_WriteRegister(MT6826S, MT6826S_ROT_REG, rotRegister);
	uint8_t check = (MT6826S_ReadRegister(MT6826S, MT6826S_ROT_REG) ^ rotRegister);

	if (check == 0) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MT6826S_ConfigABZ(MT6826STypeDef *MT6826S, uint16_t PPR, mt6826s_abz_output_t ABZ_OUTPUT, mt6826s_ab_swap_t AB_SWAP, mt6826s_z_pulse_width_t Z_WIDTH)
{
	uint8_t abzReg1 = ((PPR - 1) & 0x0FFF) >> 4;
	uint8_t abzReg2 = (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG2) & 0b00001100) | (((PPR-1) & 0xF) << 4) | (ABZ_OUTPUT << 1) | (AB_SWAP);
	uint8_t abzReg4 = (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG4) & 0xF0) | Z_WIDTH;
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG1, abzReg1);
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG2, abzReg2);
	MT6826S_WriteRegister(MT6826S, MT6826S_ABZ_REG4, abzReg4);
	uint8_t check = (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG1) ^ abzReg1) | ((MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG2) ^ abzReg2) & 0b11110011) | (MT6826S_ReadRegister(MT6826S, MT6826S_ABZ_REG4) ^ abzReg4);

	if (check == 0) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MT6826S_ConfigUVW(MT6826STypeDef *MT6826S, mt6826s_uvw_output_t UVW_OUTPUT, mt6826s_uvw_resolution_t UVW_RESOLUTION)
{
	uint8_t uvwReg = (MT6826S_ReadRegister(MT6826S, MT6826S_UVW_REG) & 0b11100000) | (UVW_OUTPUT << 4) | UVW_RESOLUTION;
	MT6826S_WriteRegister(MT6826S, MT6826S_UVW_REG, uvwReg);
	uint8_t check = MT6826S_ReadRegister(MT6826S, MT6826S_UVW_REG) ^ uvwReg;

	if (check == 0) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MT6826S_ConfigPWM(MT6826STypeDef *MT6826S, mt6826s_pwm_frame_frequency_t PWM_FRQ, mt6826s_pwm_voltage_level_t PWM_VOLTAGE, mt6826s_pwm_output_source_t PWM_SOURCE)
{
	uint8_t pwmReg = (MT6826S_ReadRegister(MT6826S, MT6826S_PWM_REG) & 0b11100000) | (PWM_FRQ << 4) | (PWM_VOLTAGE << 3) | PWM_SOURCE;
	MT6826S_WriteRegister(MT6826S, MT6826S_PWM_REG, pwmReg);
	uint8_t check = (MT6826S_ReadRegister(MT6826S, MT6826S_PWM_REG) ^ pwmReg) & 0b00011111;

	if (check == 0) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

uint16_t MT6826S_getRawAngle(MT6826STypeDef *MT6826S)
{
    uint8_t hi = MT6826S_ReadRegister(MT6826S, MT6826S_READ_ONLY1);
    uint8_t lo = MT6826S_ReadRegister(MT6826S, MT6826S_READ_ONLY2);

    return (((uint16_t)hi) << 7) | (lo >> 1);
}

float MT6826S_getAngle(MT6826STypeDef *MT6826S)
{
	float angle;
	uint16_t rawAngle;

	rawAngle = MT6826S_getRawAngle(MT6826S);
	angle = (rawAngle * 360.0f) / MT6826S_RESOLUTION;

	return angle;
}

uint8_t MT6826S_getStatus(MT6826STypeDef *MT6826S)
{
	uint8_t status;
	status = MT6826S_ReadRegister(MT6826S, MT6826S_READ_ONLY3);
	return status;
}

uint8_t MT6826S_checkStatusBit(MT6826STypeDef *MT6826S, uint8_t statusBit)
{
	uint8_t status = MT6826S_getStatus(MT6826S);
	return (status >> statusBit) & 0x01;
}

