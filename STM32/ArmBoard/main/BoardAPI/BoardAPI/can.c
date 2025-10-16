/*
 * can.c
 *
 *  Created on: Jun 8, 2025
 *      Author: Doraemonjayo
 */

#include "BoardAPI.h"

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

const CAN_FilterTypeDef CAN1_FILTER_DEFAULT = {
	.FilterIdHigh = 0,
	.FilterIdLow = 0,
	.FilterMaskIdHigh = 0,
	.FilterMaskIdLow = 0,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 0,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 14
};

const CAN_FilterTypeDef CAN2_FILTER_DEFAULT = {
	.FilterIdHigh = 0,
	.FilterIdLow = 0,
	.FilterMaskIdHigh = 0,
	.FilterMaskIdLow = 0,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 14,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 14
};

static void (*can1_receivedCallback)(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) = NULL;
static void (*can2_receivedCallback)(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) = NULL;

static uint8_t can1_buffer[8];
static uint8_t can2_buffer[8];
static CAN_RxHeaderTypeDef can1_rxHeader;
static CAN_RxHeaderTypeDef can2_rxHeader;

void can1_start(const CAN_FilterTypeDef *filter) {
	HAL_CAN_ConfigFilter(&hcan1, filter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
}

void can2_start(const CAN_FilterTypeDef *filter) {
	HAL_CAN_ConfigFilter(&hcan2, filter);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
}

uint32_t can1_txAvailable() {
	return HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

uint32_t can2_txAvailable() {
	return HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
}

void can1_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint32_t TxMailbox = 0;
	uint8_t txBuffer[8] = {0};

	if (dlc > 8) return;

	if(isExtended){
		TxHeader.ExtId = id;
	}
	else{
		TxHeader.StdId = id;
	}

	if (data != NULL && isRemote == false) {
		memcpy(txBuffer, data, dlc);
	}

	TxHeader.DLC = dlc;
	TxHeader.IDE = isExtended ? CAN_ID_EXT : CAN_ID_STD;
	TxHeader.RTR = isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	while(can1_txAvailable() == 0);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txBuffer, &TxMailbox);
}

void can2_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint32_t TxMailbox = 0;
	uint8_t txBuffer[8] = {0};

	if (dlc > 8) return;

	if(isExtended){
		TxHeader.ExtId = id;
	}
	else{
		TxHeader.StdId = id;
	}

	if (data != NULL && isRemote == false) {
		memcpy(txBuffer, data, dlc);
	}

	TxHeader.DLC = dlc;
	TxHeader.IDE = isExtended ? CAN_ID_EXT : CAN_ID_STD;
	TxHeader.RTR = isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	while(can2_txAvailable() == 0);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, txBuffer, &TxMailbox);
}

void can1_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote)) {
	can1_receivedCallback = callback;
}

void can2_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote)) {
	can2_receivedCallback = callback;
}

void (*can1_getReceivedCallback())(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	return can1_receivedCallback;
}

void (*can2_getReceivedCallback())(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	return can2_receivedCallback;
}

void can1_stop() {
	HAL_CAN_Stop(&hcan1);
}

void can2_stop() {
	HAL_CAN_Stop(&hcan2);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1) {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_rxHeader, can1_buffer);
		uint8_t ide = can1_rxHeader.IDE;
		uint32_t id = (ide == CAN_ID_STD) ? can1_rxHeader.StdId : can1_rxHeader.ExtId;
		uint8_t *data = can1_buffer;
		uint8_t dlc = can1_rxHeader.DLC;
		uint8_t rtr = can1_rxHeader.RTR;
		if(can1_receivedCallback != NULL) can1_receivedCallback(id, data, dlc, ide, rtr);
	}else if(hcan == &hcan2) {
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rxHeader, can2_buffer);
		uint8_t ide = can2_rxHeader.IDE;
		uint32_t id = (ide == CAN_ID_STD) ? can2_rxHeader.StdId : can2_rxHeader.ExtId;
		uint8_t *data = can2_buffer;
		uint8_t dlc = can2_rxHeader.DLC;
		uint8_t rtr = can2_rxHeader.RTR;
		if(can2_receivedCallback != NULL) can2_receivedCallback(id, data, dlc, ide, rtr);
	}
}
