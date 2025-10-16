/*
 * flash.h
 *
 *  Created on: Jul 3, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_FLASH_H_
#define BOARDAPI_BOARDAPI_FLASH_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

#define FLASH_USERDATA_START_ADDR	(0x08060000UL)
#define FLASH_USERDATA_SIZE			(128 * 1024UL)
#define FLASH_USERDATA				((const uint32_t *)(FLASH_USERDATA_START_ADDR))

void flash_write(const uint32_t *data, uint32_t len);
void flash_read(uint32_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_FLASH_H_ */
