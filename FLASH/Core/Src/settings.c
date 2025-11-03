/*
 * settings.c
 *
 *  Created on: Oct 3, 2025
 *      Author: andre
 */

#include "settings.h"

volatile SettingsStruct Settings;

void SettingsInit(void) {
	SettingsLoad();

	if (Settings.Parameter1 == 0xFF) { // Flash memory is clear, Settings not saved
		// Set Default Values
		Settings.Parameter1 = 1;
		Settings.Parameter2 = 253;
		Settings.Parameter3 = 72;
		Settings.Parameter4 = 16;
	}
}

void SettingsSave(void) {
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = 0;
	EraseInitStruct.Page = FLASH_PAGE_NB -1; // Last Page
	EraseInitStruct.NbPages = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)
	{
		// Write setting
		uint64_t *source_addr = (void *)&Settings;
		uint32_t dest_addr = (uint32_t) FLASH_SETTINGS_START_ADDR;
		for (uint16_t i=0; i<FLASH_SETTINGS_WORDS; i++) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest_addr, *source_addr);
			source_addr++;
			dest_addr = dest_addr + 8;
		}
	}

    HAL_FLASH_Lock();
}

void SettingsLoad(void) {
	uint32_t *source_addr = (uint32_t *)FLASH_SETTINGS_START_ADDR;
	uint32_t *dest_addr = (void *)&Settings;

	//Read settings
	for (uint16_t i=0; i<FLASH_SETTINGS_WORDS; i++) {
		*dest_addr = *(__IO uint32_t*)source_addr;
		source_addr++;
		dest_addr++;
	}
}
