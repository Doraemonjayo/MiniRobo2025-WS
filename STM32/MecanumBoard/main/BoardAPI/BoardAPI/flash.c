/*
 * flash.c
 *
 *  Created on: Jul 3, 2025
 *      Author: Doraemonjayo
 */

#include "BoardAPI.h"

void flash_write(const uint32_t *data, uint32_t len) {
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t sectorError;

	if (len > FLASH_USERDATA_SIZE / sizeof(uint32_t)) len = FLASH_USERDATA_SIZE / sizeof(uint32_t);

	HAL_FLASH_Unlock();

	eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInitStruct.Sector = FLASH_SECTOR_7;
	eraseInitStruct.NbSectors = 1;
	eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	status = HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);
	if (status != HAL_OK) {
		HAL_FLASH_Lock();
		return;
	}

	for (uint32_t i = 0; i < len; i++) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USERDATA_START_ADDR + i * sizeof(uint32_t), data[i]);
		if (status != HAL_OK) {
			HAL_FLASH_Lock();
			return;
		}
	}

	HAL_FLASH_Lock();
	return;
}

void flash_read(uint32_t *data, uint32_t len) {
	if (len > FLASH_USERDATA_SIZE / sizeof(uint32_t)) len = FLASH_USERDATA_SIZE / sizeof(uint32_t);
	memcpy(data, (const void *)FLASH_USERDATA_START_ADDR, len * sizeof(uint32_t));
}
