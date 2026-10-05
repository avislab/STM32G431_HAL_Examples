/*
 * MT6826S.h
 *
 *  Created on: Jun 2, 2026
 *      Author: andre
 */

#ifndef INC_MT6826S_H_
#define INC_MT6826S_H_


// ---------------------------------------------------------------------------
// SPI Command opcodes
// ---------------------------------------------------------------------------

#define MT6826S_READ_REG    0x3     ///< Opcode: read register
#define MT6826S_WRITE_REG   0x6     ///< Opcode: write register
#define MT6826S_EEPROM_PROG 0xC     ///< Opcode: erase and program EEPROM
#define MT6826S_READ_CONT   0xA     ///< Opcode: continuous angle register read

#define MT6826S_SUCCESS_ACK 0x55    ///< Acknowledge byte returned on successful EEPROM program

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

#define MT6826S_ID_REG      0x001   ///< User ID register
#define MT6826S_READ_ONLY1  0x003
#define MT6826S_READ_ONLY2  0x004
#define MT6826S_READ_ONLY3  0x005
#define MT6826S_READ_ONLY4  0x006
#define MT6826S_ABZ_REG1    0x007
#define MT6826S_ABZ_REG2    0x008
#define MT6826S_ABZ_REG3    0x009
#define MT6826S_ABZ_REG4    0x00A
#define MT6826S_UVW_REG     0x00B
#define MT6826S_PWM_REG     0x00C
#define MT6826S_ROT_REG     0x00D
#define MT6826S_EE_DONE_REG 0x112

// ---------------------------------------------------------------------------
// Sensor constants
// ---------------------------------------------------------------------------

#define MT6826S_RESOLUTION 32768.0f ///< Angular resolution: 2^15

// ---------------------------------------------------------------------------

/**
 * @brief Warning status bits reported in MT6826S_READ_ONLY3.
 */
typedef enum {
    MT6826S_OVER_SPEED    = 0,  ///< Speed exceeded maximum rating
    MT6826S_MAGNET_WEAK   = 1,  ///< Magnetic field strength below threshold
    MT6826S_UNDER_VOLTAGE = 2,  ///< Supply voltage below minimum rating
} mt6826s_warnings_t;

/**
 * @brief ABZ incremental output enable/disable.
 */
typedef enum {
    MT6826S_ABZ_ON    = 0,  ///< ABZ output enabled
    MT6826S_ABZ_OFF   = 1,  ///< ABZ output disabled
} mt6826s_abz_output_t;

/**
 * @brief Swap A and B output channels.
 */
typedef enum {
    MT6826S_NO_SWAP = 0,    ///< A/B channels in default order
    MT6826S_SWAP    = 1,    ///< A and B channels swapped
} mt6826s_ab_swap_t;

/**
 * @brief Z index pulse width selection.
 *
 * LSB_n values define pulse width in encoder counts. 1 LSB = 360/(PPR*4) degrees.
 * ANGLE_n values define pulse width as a fixed angular span.
 */
typedef enum {
    MT6826S_LSB_1     = 0x0,    ///< Z pulse width = 1 LSB
    MT6826S_LSB_2     = 0x1,    ///< Z pulse width = 2 LSB
    MT6826S_LSB_4     = 0x2,    ///< Z pulse width = 4 LSB
    MT6826S_LSB_8     = 0x3,    ///< Z pulse width = 8 LSB
    MT6826S_LSB_16    = 0x4,    ///< Z pulse width = 16 LSB
    MT6826S_ANGLE_60  = 0x5,    ///< Z pulse width = 60 degrees
    MT6826S_ANGLE_120 = 0x6,    ///< Z pulse width = 120 degrees
    MT6826S_ANGLE_180 = 0x7,    ///< Z pulse width = 180 degrees
    MT6826S_LSB_32    = 0x8,    ///< Z pulse width = 32 LSB
    MT6826S_LSB_64    = 0x9,    ///< Z pulse width = 64 LSB
    MT6826S_LSB_128   = 0xA,    ///< Z pulse width = 128 LSB
    MT6826S_ANGLE_45  = 0xB,    ///< Z pulse width = 45 degrees
    MT6826S_ANGLE_90  = 0xC,    ///< Z pulse width = 90 degrees
    MT6826S_ANGLE_135 = 0xD,    ///< Z pulse width = 135 degrees
    MT6826S_ANGLE_240 = 0xE,    ///< Z pulse width = 240 degrees
} mt6826s_z_pulse_width_t;

/**
 * @brief UVW commutation output enable/disable.
 */
typedef enum {
    MT6826S_UVW_ON    = 0,  ///< UVW output enabled
    MT6826S_UVW_OFF   = 1,  ///< UVW output disabled
} mt6826s_uvw_output_t;

/**
 * @brief UVW pole pair count selection.
 *
 * Defines the number of cycles per mechanical revolution
 * for the UVW commutation output.
 */
typedef enum {
    MT6826S_POLE_PAIRS_1  = 0x0,    ///< 1 pole pair
    MT6826S_POLE_PAIRS_2  = 0x1,    ///< 2 pole pairs
    MT6826S_POLE_PAIRS_3  = 0x2,    ///< 3 pole pairs
    MT6826S_POLE_PAIRS_4  = 0x3,    ///< 4 pole pairs
    MT6826S_POLE_PAIRS_5  = 0x4,    ///< 5 pole pairs
    MT6826S_POLE_PAIRS_6  = 0x5,    ///< 6 pole pairs
    MT6826S_POLE_PAIRS_7  = 0x6,    ///< 7 pole pairs
    MT6826S_POLE_PAIRS_8  = 0x7,    ///< 8 pole pairs
    MT6826S_POLE_PAIRS_9  = 0x8,    ///< 9 pole pairs
    MT6826S_POLE_PAIRS_10 = 0x9,    ///< 10 pole pairs
    MT6826S_POLE_PAIRS_11 = 0xA,    ///< 11 pole pairs
    MT6826S_POLE_PAIRS_12 = 0xB,    ///< 12 pole pairs
    MT6826S_POLE_PAIRS_13 = 0xC,    ///< 13 pole pairs
    MT6826S_POLE_PAIRS_14 = 0xD,    ///< 14 pole pairs
    MT6826S_POLE_PAIRS_15 = 0xE,    ///< 15 pole pairs
    MT6826S_POLE_PAIRS_16 = 0xF,    ///< 16 pole pairs
} mt6826s_uvw_resolution_t;

/**
 * @brief PWM output frame frequency selection.
 */
typedef enum {
    MT6826S_PWM_994_HZ = 0, ///< PWM frame frequency = 994 Hz
    MT6826S_PWM_497_HZ = 1, ///< PWM frame frequency = 497 Hz
} mt6826s_pwm_frame_frequency_t;

/**
 * @brief PWM output active voltage level.
 */
typedef enum {
    MT6826S_HIGH_EFFECTIVE  = 0,    ///< PWM active high
    MT6826S_LOW_EFFECTIVE   = 1,    ///< PWM active low
} mt6826s_pwm_voltage_level_t;

/**
 * @brief PWM output data source selection.
 */
typedef enum {
    MT6826S_ANGLE_12BIT  = 0x0, ///< PWM encodes 12-bit angle
    MT6826S_SPEED_12BIT  = 0x2, ///< PWM encodes 12-bit speed
} mt6826s_pwm_output_source_t;

/**
 * @brief Shaft rotation direction convention.
 */
typedef enum {
    MT6826S_COUNTER_CLOCKWISE = 0,  ///< Angle increases counter-clockwise
    MT6826S_CLOCKWISE         = 1,  ///< Angle increases clockwise
} mt6826s_rotation_direction_t;

/**
 * @brief Zero position reference for setZeroPosition().
 */
typedef enum {
    MT6826S_ANGLE_0         = 0,    ///< Set zero position to absolute 0 degrees
    MT6826S_CURRENT_ANGLE   = 1,    ///< Set zero position to current shaft angle
} mt6826s_zero_pos_t;


typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *CSNPort;
	uint16_t CSNPin;
} MT6826STypeDef;


void MT6826S_Init(MT6826STypeDef *MT6826S, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CSNPort, uint16_t CSNPin);
uint8_t MT6826S_ReadRegister(MT6826STypeDef *MT6826S, uint16_t address);
HAL_StatusTypeDef MT6826S_WriteRegister(MT6826STypeDef *MT6826S, uint16_t address, uint8_t data);

HAL_StatusTypeDef MT6826S_ProgramEEPROM(MT6826STypeDef *MT6826S);
HAL_StatusTypeDef MT6826S_ConfigZeroPosition(MT6826STypeDef *MT6826S, mt6826s_zero_pos_t ZERO_POS);
HAL_StatusTypeDef MT6826S_ConfigSpecificZeroPosition(MT6826STypeDef *MT6826S, uint16_t angle12Bit);
HAL_StatusTypeDef MT6826S_ConfigDirection(MT6826STypeDef *MT6826S, mt6826s_rotation_direction_t ROTATION_DIRECTION);
HAL_StatusTypeDef MT6826S_ConfigABZ(MT6826STypeDef *MT6826S, uint16_t PPR, mt6826s_abz_output_t ABZ_OUTPUT, mt6826s_ab_swap_t AB_SWAP, mt6826s_z_pulse_width_t Z_WIDTH);
HAL_StatusTypeDef MT6826S_ConfigUVW(MT6826STypeDef *MT6826S, mt6826s_uvw_output_t UVW_OUTPUT, mt6826s_uvw_resolution_t UVW_RESOLUTION);
HAL_StatusTypeDef MT6826S_ConfigPWM(MT6826STypeDef *MT6826S, mt6826s_pwm_frame_frequency_t PWM_FRQ, mt6826s_pwm_voltage_level_t PWM_VOLTAGE, mt6826s_pwm_output_source_t PWM_SOURCE);

uint16_t MT6826S_getRawAngle(MT6826STypeDef *MT6826S);
float MT6826S_getAngle(MT6826STypeDef *MT6826S);

uint8_t MT6826S_getStatus(MT6826STypeDef *MT6826S);
uint8_t MT6826S_checkStatusBit(MT6826STypeDef *MT6826S, uint8_t statusBit);

#endif /* INC_MT6826S_H_ */
