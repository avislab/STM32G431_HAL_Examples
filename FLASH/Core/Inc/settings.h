/*
 * settings.h
 *
 *  Created on: Oct 3, 2025
 *      Author: andre
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "main.h"

#define FLASH_SETTINGS_START_ADDR	0x801F800  // Last Page address for G431

typedef struct
{
	char Parameter1;		// 1 byte
	uint8_t Parameter2;		// 1 byte
	uint16_t Parameter3;	// 2 byte
	uint32_t Parameter4;	// 4 byte
} SettingsStruct;


#define FLASH_SETTINGS_WORDS 					(sizeof(SettingsStruct)/4)

void SettingsInit(void);
void SettingsSave(void);
void SettingsLoad(void);

#endif /* INC_SETTINGS_H_ */
