/*
 * MS5611.h
 *
 *  Created on: 12 окт. 2025 г.
 *      Author: andrey
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#define DEVICE_ADDRESS 0x76
//OSR 0 - Resolution 0.065 mbar, Conversion time 0.60 ms
//OSR 1 - Resolution 0.042 mbar, Conversion time 1.17 ms
//OSR 2 - Resolution 0.027 mbar, Conversion time 2.28 ms
//OSR 3 - Resolution 0.018 mbar, Conversion time 4.54 ms
//OSR 4 - Resolution 0.012 mbar, Conversion time 9.04 ms

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint8_t address;
	uint8_t OSR;

	uint16_t C1, C2, C3, C4, C5, C6;

	uint8_t buf[3];

} MS5611TypeDef;

void MS5611_Init(MS5611TypeDef *MS5611, I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t OSR);
void MS5611_Convert(MS5611TypeDef *MS5611, long* temperature, long* pressure);

#endif /* INC_MS5611_H_ */
