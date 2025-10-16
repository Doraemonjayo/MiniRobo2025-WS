/*
 * cdc.h
 *
 *  Created on: May 19, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_CDC_H_
#define BOARDAPI_BOARDAPI_CDC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

void cdc_setReceivedCallback(void (*callback)(uint8_t *buf, uint32_t len));
void (*cdc_getReceivedCallback(void))(uint8_t *buf, uint32_t len);
uint32_t cdc_txAvailable();
uint8_t cdc_isConnected();
void cdc_transmit(const uint8_t *buf, uint32_t len);
void cdc_startReceive();
const char *cdc_getProductName();
const char *cdc_getVendorName();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_CDC_H_ */
