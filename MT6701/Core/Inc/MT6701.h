/*
 * MT6701.h
 *
 *  Created on: 30 мая 2026 г.
 *      Author: andrey
 */

#ifndef INC_MT6701_H_
#define INC_MT6701_H_

#define MT6701_DEFAULT_ADDRESS				0x06

#define MT6701_OK							0
#define MT6701_ERR_GENERAL					1
#define MT6701_ERR_HANDLER_NULL				2
#define MT6701_ERR_CONFIG_UNAVAILABLE		3
#define MT6701_ERR_IO						4
#define MT6701_ERR_OUT_OF_RANGE				5
#define MT6701_ERR_UNINITITIALIZED			6

#define MT6701_REG_ANGLE0					0x04
#define MT6701_REG_ANGLE6					0x03
#define MT6701_REG_UVM_MUX					0x25
#define MT6701_REG_ABZ_MUX					0x29
#define MT6701_REG_DIR						0x29
#define MT6701_REG_UVW_RES0					0x30
#define MT6701_REG_ABZ_RES8					0x30
#define MT6701_REG_ABZ_RES0					0x31
#define MT6701_REG_ZERO8					0x32
#define MT6701_REG_PULSE_WIDTH				0x32
#define MT6701_REG_HYST2					0x32
#define MT6701_REG_ZERO0					0x33
#define MT6701_REG_HYST0					0x34
#define MT6701_REG_PWM_FREQ					0x38
#define MT6701_REG_PWM_POL					0x38
#define MT6701_REG_OUT_MODE					0x38
#define MT6701_REG_A_STOP8					0x3E
#define MT6701_REG_A_START8					0x3E
#define MT6701_REG_A_START0					0x3F
#define MT6701_REG_A_STOP0					0x40


#define MT6701_REG_ANGLE0_POS				2
#define MT6701_REG_ANGLE6_POS				0
#define MT6701_REG_UVM_MUX_POS				7
#define MT6701_REG_ABZ_MUX_POS				6
#define MT6701_REG_DIR_POS					1
#define MT6701_REG_UVW_RES0_POS				4
#define MT6701_REG_ABZ_RES8_POS				0
#define MT6701_REG_ABZ_RES0_POS				0
#define MT6701_REG_ZERO8_POS				0
#define MT6701_REG_PULSE_WIDTH_POS			4
#define MT6701_REG_HYST2_POS				7
#define MT6701_REG_ZERO0_POS				0
#define MT6701_REG_HYST0_POS				6
#define MT6701_REG_PWM_FREQ_POS				7
#define MT6701_REG_PWM_POL_POS				6
#define MT6701_REG_OUT_MODE_POS				5
#define MT6701_REG_A_STOP8_POS				4
#define MT6701_REG_A_START8_POS				0
#define MT6701_REG_A_START0_POS				0
#define MT6701_REG_A_STOP0_POS				0


#define MT6701_REG_ANGLE0_MASK				(0x3F << MT6701_REG_ANGLE0_POS)
#define MT6701_REG_ANGLE6_MASK				(0xFF << MT6701_REG_ANGLE6_POS)
#define MT6701_REG_UVM_MUX_MASK				(0x01 << MT6701_REG_UVM_MUX_POS)
#define MT6701_REG_ABZ_MUX_MASK				(0x01 << MT6701_REG_ABZ_MUX_POS)
#define MT6701_REG_DIR_MASK					(0x01 << MT6701_REG_DIR_POS)
#define MT6701_REG_UVW_RES0_MASK			(0x0F << MT6701_REG_UVW_RES0_POS)
#define MT6701_REG_ABZ_RES8_MASK			(0x03 << MT6701_REG_ABZ_RES8_POS)
#define MT6701_REG_ABZ_RES0_MASK			(0xFF << MT6701_REG_ABZ_RES0_POS)
#define MT6701_REG_ZERO8_MASK				(0x0F << MT6701_REG_ZERO8_POS)
#define MT6701_REG_PULSE_WIDTH_MASK			(0x07 << MT6701_REG_PULSE_WIDTH_POS)
#define MT6701_REG_HYST2_MASK				(0x01 << MT6701_REG_HYST2_POS)
#define MT6701_REG_ZERO0_MASK				(0xFF << MT6701_REG_ZERO0_POS)
#define MT6701_REG_HYST0_MASK				(0x03 << MT6701_REG_HYST0_POS)
#define MT6701_REG_PWM_FREQ_MASK			(0x01 << MT6701_REG_PWM_FREQ_POS)
#define MT6701_REG_PWM_POL_MASK				(0x01 << MT6701_REG_PWM_POL_POS)
#define MT6701_REG_OUT_MODE_MASK			(0x01 << MT6701_REG_OUT_MODE_POS)
#define MT6701_REG_A_STOP8_MASK				(0x0F << MT6701_REG_A_STOP8_POS)
#define MT6701_REG_A_START8_MASK			(0x0F << MT6701_REG_A_START8_POS)
#define MT6701_REG_A_START0_MASK			(0xFF << MT6701_REG_A_START0_POS)
#define MT6701_REG_A_STOP0_MASK				(0xFF << MT6701_REG_A_STOP0_POS)

typedef enum{
	MT6701_MODE_NONE,
	MT6701_MODE_UVW,
	MT6701_MODE_ABZ,
} mt6701_mode_t;

typedef enum{
	MT6701_HYST_1		= 0x0,
	MT6701_HYST_2		= 0x1,
	MT6701_HYST_4		= 0x2,
	MT6701_HYST_8		= 0x3,
	MT6701_HYST_0_25	= 0x5,
	MT6701_HYST_0_5		= 0x6,
} mt6701_hyst_t;

/*
typedef enum{
    MT6701_INTERFACE_NONE,
	MT6701_INTERFACE_I2C,
	MT6701_INTERFACE_SSI,
} mt6701_interface_t;
*/

typedef enum{
	MT6701_DIRECTION_CW		= 0x0,
	MT6701_DIRECTION_CCW	= 0x1,
} mt6701_direction_t;

typedef enum{
	MT6701_PWM_FREQ_994_4	= 0x0,
	MT6701_PWM_FREQ_497_2	= 0x1,
} mt6701_pwm_freq_t;

typedef enum{
	MT6701_PWM_POL_HIGH		= 0x0,
	MT6701_PWM_POL_LOW		= 0x1,
} mt6701_pwm_pol_t;

typedef enum{
	MT6701_OUT_MODE_ANALOG	= 0x0,
	MT6701_OUT_MODE_PWM		= 0x1,
} mt6701_out_mode_t;

typedef enum{
	MT6701_PULSE_WIDTH_1LSB 	= 0x0,
	MT6701_PULSE_WIDTH_2LSB 	= 0x1,
	MT6701_PULSE_WIDTH_4LSB 	= 0x2,
	MT6701_PULSE_WIDTH_8LSB 	= 0x3,
	MT6701_PULSE_WIDTH_12LSB 	= 0x4,
	MT6701_PULSE_WIDTH_16LSB 	= 0x5,
	MT6701_PULSE_WIDTH_180 		= 0x6,
} mt6701_pulse_width_t;

typedef enum{
	MT6701_STATUS_NORM			= 0x0,
	MT6701_STATUS_FIELD_STRONG	= 0x1,
	MT6701_STATUS_FIELD_WEAK	= 0x2,
	MT6701_STATUS_FIELD_ERROR	= 0x3,
} mt6701_status_t;

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint8_t address;
} MT6701TypeDef;

void MT6701_Init(MT6701TypeDef *MT6701, I2C_HandleTypeDef *hi2c, uint8_t address);
//uint8_t MT6701_nanbnz_enable( MT6701TypeDef *MT6701, uint8_t nanbnz_enable);
uint8_t MT6701_abz_pulse_per_round_set(MT6701TypeDef *MT6701, uint16_t resolution);
uint8_t MT6701_uvw_pole_pair_set(MT6701TypeDef *MT6701, uint8_t pole_pairs);
uint8_t MT6701_mode_set(MT6701TypeDef *MT6701, mt6701_mode_t mode);
uint8_t MT6701_zero_set_raw(MT6701TypeDef *MT6701, uint16_t zero_angle);
uint8_t MT6701_zero_set(MT6701TypeDef *MT6701, float zero_angle);
uint8_t MT6701_hyst_set(MT6701TypeDef *MT6701, mt6701_hyst_t hysteresis);
uint8_t MT6701_a_start_stop_set_raw(MT6701TypeDef *MT6701, uint16_t start, uint16_t stop);
uint8_t MT6701_a_start_stop_set(MT6701TypeDef *MT6701, float start, float stop);
uint8_t mt6701_direction_set(MT6701TypeDef *MT6701, mt6701_direction_t direction);
uint8_t MT6701_pulse_width_set(MT6701TypeDef *MT6701, mt6701_pulse_width_t pulse_width);
uint8_t mt6701_pwm_freq_set(MT6701TypeDef *MT6701, mt6701_pwm_freq_t pwm_freq);
uint8_t MT6701_pwm_polarity_set(MT6701TypeDef *MT6701, mt6701_pwm_pol_t pwm_polarity);
uint8_t MT6701_out_mode_set(MT6701TypeDef *MT6701, mt6701_out_mode_t out_mode);
uint8_t MT6701_programm_eeprom(MT6701TypeDef *MT6701);
uint8_t MT6701_read_raw(MT6701TypeDef *MT6701, uint16_t *angle_raw);
uint8_t MT6701_read(MT6701TypeDef *MT6701, float *angle);

#endif /* INC_MT6701_H_ */
