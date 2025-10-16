/*
 * cdc.c
 *
 *  Created on: May 19, 2025
 *      Author: Doraemonjayo
 */

#include "BoardAPI.h"

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif /* MAX */

static void (*receivedCallback)(uint8_t *buf, uint32_t len) = NULL;

void cdc_setReceivedCallback(void (*callback)(uint8_t *buf, uint32_t len)) {
	receivedCallback = callback;
}

void (*cdc_getReceivedCallback(void))(uint8_t *buf, uint32_t len) {
	return receivedCallback;
}

uint32_t cdc_txAvailable() {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	if (hcdc == NULL) {
		return 0;
	}
	return hcdc->TxState == 0 ? 1 : 0;
}

uint8_t cdc_isConnected() {
	if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
		return 0;
	}

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	if (hcdc == NULL) {
		return 0;
	}

	return 1;
}

void cdc_transmit(const uint8_t *buf, uint32_t len) {
	len = MIN(len, 64);
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	if (hcdc == NULL) {
		return;
	}
	uint8_t _buf[64] = {0};
	memcpy(_buf, buf, len);

	while (cdc_txAvailable() == 0);
	CDC_Transmit_HS(_buf, len);
}

__weak const char *cdc_getProductName() {
	return "Undefined";
}

__weak const char *cdc_getVendorName() {
	return "DoradoraRobot";
}

void cdc_startReceive() {
	USBD_CDC_ReceivePacket(&hUsbDeviceHS);
}
