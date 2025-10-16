/*
 * can.h
 *
 *  Created on: Jun 8, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_CAN_H_
#define BOARDAPI_BOARDAPI_CAN_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern const CAN_FilterTypeDef CAN1_FILTER_DEFAULT;
extern const CAN_FilterTypeDef CAN2_FILTER_DEFAULT;

void can1_start(const CAN_FilterTypeDef *filter);
void can2_start(const CAN_FilterTypeDef *filter);

uint32_t can1_txAvailable();
uint32_t can2_txAvailable();

void can1_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
void can2_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);

void can1_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote));
void can2_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote));

void (*can1_getReceivedCallback())(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
void (*can2_getReceivedCallback())(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);

void can1_stop();
void can2_stop();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_CAN_H_ */
