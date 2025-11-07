/*
 * main_app.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#include "main_app.h"

#define CAN_QUEUE_CAPACITY 64

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t dlc;
	bool isExtended;
	bool isRemote;
} CanPacket;

static RoboMasters robomasters;

static Queue can1_txQueue;
static CanPacket can1_txBuffer[CAN_QUEUE_CAPACITY];
static Queue can2_txQueue;
static CanPacket can2_txBuffer[CAN_QUEUE_CAPACITY];

static void task1kHz();
static void can1_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can2_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can1_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can2_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);

static CanPacket can2_rxPacket = {0};

void setup() {
	gpio_setLedG(GPIO_PIN_SET);

	timer_startUs();

	RoboMasters_setTxFunc(&robomasters, can1_transmitQueue);
	RoboMasterConfig rmConf;
	rmConf.model = ROBOMASTER_M2006;
	PID_init(&rmConf.pidVelocity, ROBOMASTER_M2006_PID_VELOCITY_DEFAULT);
	PID_init(&rmConf.pidPosition, ROBOMASTER_M2006_PID_POSITION_DEFAULT);
	rmConf.controlTimeout = 200;	 // [ms]
	rmConf.feedbackTimeout = 200; // [ms]
	rmConf.rawPositionOffset = 0;
	rmConf.trapezoidAcceleration = 36.0f * 2.0f * PI;
	rmConf.trapezoidMaxAcceleration = rmConf.trapezoidAcceleration * 1.5f;
	rmConf.trapezoidDeadband = PI / 8.0f;
	for (uint8_t i = 0; i < 8; i++) {
		RoboMaster_init(&robomasters.robomaster[i], rmConf);
		RoboMaster_setCurrentLimit(&robomasters.robomaster[i],
				-1.0f, 1.0f);
		RoboMaster_setVelocityLimit(&robomasters.robomaster[i],
				-36.0f * 2.0f * PI, 36.0f * 2.0f * PI); // [rad/s]
		RoboMaster_setPositionLimit(&robomasters.robomaster[i],
				-INFINITY, INFINITY); // [rad]
	}

	RoboMaster_setCurrentLimit(&robomasters.robomaster[0], -ROBOMASTER_M2006_MAX_CURRENT, ROBOMASTER_M2006_MAX_CURRENT);

	Queue_init(&can1_txQueue, can1_txBuffer, sizeof(CanPacket), CAN_QUEUE_CAPACITY, false, disable_irq_nest, enable_irq_nest);
	Queue_init(&can2_txQueue, can2_txBuffer, sizeof(CanPacket), CAN_QUEUE_CAPACITY, false, disable_irq_nest, enable_irq_nest);

	HAL_Delay(500);

	can1_setReceivedCallback(can1_rxCallback);
	can2_setReceivedCallback(can2_rxCallback);
	can1_start(&CAN1_FILTER_DEFAULT);
	can2_start(&CAN2_FILTER_DEFAULT);

	HAL_Delay(500);
	for (uint8_t i = 0; i < 8; i++) {
		RoboMaster_resetZeroPosition(&robomasters.robomaster[i]);
	}

	timer_set1kHzTask(task1kHz);
	timer_start1kHzTask();

	UNUSED(can2_transmitQueue);

	gpio_setLedG(GPIO_PIN_RESET);
}

void loop() {
	CanPacket canPacket;
	if (Queue_size(&can1_txQueue) > 0 && can1_txAvailable() > 0) {
		if (Queue_pop(&can1_txQueue, &canPacket) == 0) {
			can1_transmit(canPacket.id, canPacket.data, canPacket.dlc, canPacket.isExtended, canPacket.isRemote);
		}
	}
	if (Queue_size(&can2_txQueue) > 0 && can2_txAvailable() > 0) {
		if (Queue_pop(&can2_txQueue, &canPacket) == 0) {
			can2_transmit(canPacket.id, canPacket.data, canPacket.dlc, canPacket.isExtended, canPacket.isRemote);
		}
	}
}

static void task1kHz() {
	static uint32_t tick = 0;

	for (uint8_t i = 0; i < 8; i++) {
		RoboMaster_calculateOutputCurrent(&robomasters.robomaster[i]);
	}
	RoboMaster_transmit(&robomasters);

	gpio_setLedR((tick % 1000 < 500) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	tick++;
}

static void can1_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	RoboMaster_rxTask(&robomasters, id, data, dlc, isExtended, isRemote);
}

static void can2_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	can2_rxPacket.id = id;
	memcpy(can2_rxPacket.data, data, dlc);
	can2_rxPacket.dlc = dlc;
	can2_rxPacket.isExtended = isExtended;
	can2_rxPacket.isRemote = isRemote;
	if (id == 2 && dlc == 8 && isExtended == false && isRemote == false) {
		float armPositions[2];
		memcpy(armPositions, data, 8);
		for (uint8_t i = 0; i < 2; i++) {
			RoboMaster_setTargetTrapezoid(&robomasters.robomaster[i], armPositions[i] * 36.0f * 2.0f * PI);
		}
	}
}

static void can1_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (dlc > 8) return;

	CanPacket packet;
	packet.id = id;
	memcpy(packet.data, data, dlc);
	packet.dlc = dlc;
	packet.isExtended = isExtended;
	packet.isRemote = isRemote;

	Queue_push(&can1_txQueue, &packet);
}

static void can2_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (dlc > 8) return;

	CanPacket packet;
	packet.id = id;
	memcpy(packet.data, data, dlc);
	packet.dlc = dlc;
	packet.isExtended = isExtended;
	packet.isRemote = isRemote;

	Queue_push(&can2_txQueue, &packet);
}
