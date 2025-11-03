/*
 * BMP280.h
 *
 *  Created on: 13 окт. 2025 г.
 *      Author: andrey
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#define DEVICE_ADDRESS 0x76

#define	BMP280_REG_CONTROL 0xF4
#define	BMP280_REG_CONFIG 0xF5

#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.

#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0

#define	BMP280_OVERSAMPLING_T1		0x20
#define	BMP280_OVERSAMPLING_T2		0x40
#define	BMP280_OVERSAMPLING_T4		0x60
#define	BMP280_OVERSAMPLING_T8		0x80
#define	BMP280_OVERSAMPLING_T16		0xA0

#define	BMP280_OVERSAMPLING_P1		0x04
#define	BMP280_OVERSAMPLING_P2		0x08
#define	BMP280_OVERSAMPLING_P4		0x0C
#define	BMP280_OVERSAMPLING_P8		0x10
#define	BMP280_OVERSAMPLING_P16		0x14

#define	BMP280_MODE_SLEEP			0x00
#define	BMP280_MODE_FORCED			0x01
#define	BMP280_MODE_NORMAL			0x03

#define	BMP280_TSB_0_5				0x00
#define	BMP280_TSB_62_5				0x20
#define	BMP280_TSB_125				0x40
#define	BMP280_TSB_250				0x60
#define	BMP280_TSB_500				0x80
#define	BMP280_TSB_1000				0xA0
#define	BMP280_TSB_2000				0xC0
#define	BMP280_TSB_4000				0xE0

#define	BMP280_FILTER_OFF			0x00
#define	BMP280_FILTER_COEFFICIENT2	0x04
#define	BMP280_FILTER_COEFFICIENT4	0x08
#define	BMP280_FILTER_COEFFICIENT8	0x0C
#define	BMP280_FILTER_COEFFICIENT16	0x10

#define	BMP280_SPI_OFF	0x00
#define	BMP280_SPI_ON	0x01

#define	BMP280_MEAS			(BMP280_OVERSAMPLING_T16 | BMP280_OVERSAMPLING_P16 | BMP280_MODE_NORMAL)
#define	BMP280_CONFIG		(BMP280_TSB_0_5 | BMP280_FILTER_COEFFICIENT16 | BMP280_SPI_OFF)

#define BMP280_CHIP_ID  0x58
#define BME280_CHIP_ID  0x60

#define BMP280_RESET_VALUE     0xB6
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint8_t address;
	uint8_t id;

	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t buf[3];

} BMP280TypeDef;

void BMP280_Init(BMP280TypeDef *BMP280, I2C_HandleTypeDef *hi2c, uint8_t address);
void BMP280_Convert(BMP280TypeDef *BMP280, int32_t* temperature, int32_t* pressure);

#endif /* INC_BMP280_H_ */
