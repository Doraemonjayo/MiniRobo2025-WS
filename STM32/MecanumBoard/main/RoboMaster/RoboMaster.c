/*
 * RoboMaster.c
 *
 *  Created on: Jun 14, 2025
 *      Author: Doraemonjayo
 */

#include "RoboMaster.h"

#define PI ((float)M_PI)

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif /* MAX */

const PIDConfig ROBOMASTER_M2006_PID_VELOCITY_DEFAULT = {
	0.05f,
	0.5f,
	0.0f,
	0.001f,
	-INFINITY,
	INFINITY,
	-ROBOMASTER_M2006_MAX_CURRENT,
	ROBOMASTER_M2006_MAX_CURRENT,
	-ROBOMASTER_M2006_MAX_CURRENT,
	ROBOMASTER_M2006_MAX_CURRENT,
	-INFINITY,
	INFINITY,
	0.0f,
	0.0f,
};

const PIDConfig ROBOMASTER_M2006_PID_POSITION_DEFAULT = {
	50.0f,
	0.0f,
	2.0f,
	0.001f,
	-INFINITY,
	INFINITY,
	-INFINITY,
	INFINITY,
	-INFINITY,
	INFINITY,
	-INFINITY,
	INFINITY,
	0.0f,
	0.0f,
};

const PIDConfig ROBOMASTER_M3508_PID_VELOCITY_DEFAULT = {
	0.0f,
	0.0f,
	0.0f,
	0.001f,
	-INFINITY,
	INFINITY,
	-ROBOMASTER_M3508_MAX_CURRENT,
	ROBOMASTER_M3508_MAX_CURRENT,
	-ROBOMASTER_M3508_MAX_CURRENT,
	ROBOMASTER_M3508_MAX_CURRENT,
	-INFINITY,
	INFINITY,
	0.0f,
	0.0f,
};

const PIDConfig ROBOMASTER_M3508_PID_POSITION_DEFAULT = {
	0.0f,
	0.0f,
	0.0f,
	0.001f,
	-INFINITY,
	INFINITY,
	-INFINITY,
	INFINITY,
	-INFINITY,
	INFINITY,
	-INFINITY,
	INFINITY,
	0.0f,
	0.0f,
};

void RoboMaster_resetFeedback(RoboMaster *robomaster) {
	robomaster->feedback.rawCurrent = 0;
	robomaster->feedback.rawVelocity = 0;
	robomaster->feedback.rawPosition = 0;
	robomaster->feedback.lastRawPosition = 0;
	robomaster->feedback.roundCount = 0;

	robomaster->feedback.current = 0.0f;
	robomaster->feedback.velocity = 0.0f;
	robomaster->feedback.position = 0.0f;

	robomaster->feedback.lastFeedbackTime = 0;
	robomaster->feedback.lastControlTime = 0;
}

void RoboMaster_stop(RoboMaster *robomaster) {
	PID_reset(&robomaster->config.pidPosition);
	PID_reset(&robomaster->config.pidVelocity);
	robomaster->mode = ROBOMASTER_MODE_CURRENT;
	robomaster->target = 0.0f;
	robomaster->outputCurrent = 0.0f;
}

void RoboMaster_reset(RoboMaster *robomaster) {
	RoboMaster_resetFeedback(robomaster);
	RoboMaster_stop(robomaster);
}

void RoboMaster_init(RoboMaster *robomaster, RoboMasterConfig config) {
	robomaster->config = config;
	RoboMaster_reset(robomaster);
}

void RoboMasters_setTxFunc(RoboMasters *robomasters, void (*canTxFunc)(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote)) {
	robomasters->canTxFunc = canTxFunc;
}

void RoboMaster_setModel(RoboMaster *robomaster, RoboMasterModel model) {
	robomaster->config.model = model;
}

void RoboMaster_setMode(RoboMaster *robomaster, RoboMasterMode mode) {
	if (robomaster->mode == mode) return;

	robomaster->mode = mode;
	switch (mode) {
	case ROBOMASTER_MODE_CURRENT:
		PID_reset(&robomaster->config.pidPosition);
		PID_reset(&robomaster->config.pidVelocity);
		break;
	case ROBOMASTER_MODE_VELOCITY:
		PID_reset(&robomaster->config.pidPosition);
		break;
	case ROBOMASTER_MODE_POSITION:
		break;
	case ROBOMASTER_MODE_TRAPEZOID:
		PID_reset(&robomaster->config.pidPosition);
		break;
	default:
		RoboMaster_stop(robomaster);
		return;
	}
}

void RoboMaster_setControlTimeout(RoboMaster *robomaster, uint32_t timeout) {
	robomaster->config.controlTimeout = timeout;
}

void RoboMaster_setFeedbackTimeout(RoboMaster *robomaster, uint32_t timeout) {
	robomaster->config.feedbackTimeout = timeout;
}

void RoboMaster_setTimeout(RoboMaster *robomaster, uint32_t controlTimeout, uint32_t feedbackTimeout) {
	RoboMaster_setControlTimeout(robomaster, controlTimeout);
	RoboMaster_setFeedbackTimeout(robomaster, feedbackTimeout);
}

void RoboMaster_setCurrentLimit(RoboMaster *robomaster, float minCurrent, float maxCurrent) {
	robomaster->config.pidVelocity.config.minOut = minCurrent;
	robomaster->config.pidVelocity.config.maxOut = maxCurrent;
}

void RoboMaster_setVelocityLimit(RoboMaster *robomaster, float minVelocity, float maxVelocity) {
	robomaster->config.pidVelocity.config.minTarget = minVelocity;
	robomaster->config.pidVelocity.config.maxTarget = maxVelocity;
	robomaster->config.pidPosition.config.minOut = minVelocity;
	robomaster->config.pidPosition.config.maxOut = maxVelocity;
}

void RoboMaster_setPositionLimit(RoboMaster *robomaster, float minPosition, float maxPosition) {
	robomaster->config.pidPosition.config.minTarget = minPosition;
	robomaster->config.pidPosition.config.maxTarget = maxPosition;
}

void RoboMaster_setTrapezoidAcceleration(RoboMaster *robomaster, float acceleration) {
	robomaster->config.trapezoidAcceleration = acceleration;
}

void RoboMaster_setTrapezoidMaxAcceleration(RoboMaster *robomaster, float acceleration) {
	robomaster->config.trapezoidMaxAcceleration = acceleration;
}

void RoboMaster_setRoundCount(RoboMaster *robomaster, int32_t roundCount) {
	robomaster->feedback.roundCount = roundCount;
}

void RoboMaster_setRawPositionOffset(RoboMaster *robomaster, int16_t offset) {
	robomaster->config.rawPositionOffset = offset;
	robomaster->feedback.position = (robomaster->feedback.roundCount * ROBOMASTER_ENCODER_RESOLUTION + robomaster->feedback.rawPosition - robomaster->config.rawPositionOffset) * (2.0f * PI / ROBOMASTER_ENCODER_RESOLUTION);
}

void RoboMaster_resetZeroPosition(RoboMaster *robomaster) {
	RoboMaster_setRoundCount(robomaster, 0);
	RoboMaster_setRawPositionOffset(robomaster, robomaster->feedback.rawPosition);
}

void RoboMaster_setTargetCurrent(RoboMaster *robomaster, float current) {
	RoboMaster_setMode(robomaster, ROBOMASTER_MODE_CURRENT);
	robomaster->target = current;
	robomaster->feedback.lastControlTime = HAL_GetTick();
}

void RoboMaster_setTargetVelocity(RoboMaster *robomaster, float velocity) {
	RoboMaster_setMode(robomaster, ROBOMASTER_MODE_VELOCITY);
	robomaster->target = velocity;
	robomaster->feedback.lastControlTime = HAL_GetTick();
}

void RoboMaster_setTargetPosition(RoboMaster *robomaster, float position) {
	RoboMaster_setMode(robomaster, ROBOMASTER_MODE_POSITION);
	robomaster->target = position;
	robomaster->feedback.lastControlTime = HAL_GetTick();
}

void RoboMaster_setTargetTrapezoid(RoboMaster *robomaster, float position) {
	RoboMaster_setMode(robomaster, ROBOMASTER_MODE_TRAPEZOID);
	robomaster->target = position;
	robomaster->feedback.lastControlTime = HAL_GetTick();
}

void RoboMaster_calculateOutputCurrent(RoboMaster *robomaster) {
	if (HAL_GetTick() > robomaster->feedback.lastControlTime + robomaster->config.controlTimeout) {
		RoboMaster_stop(robomaster);
		return;
	}
	if (HAL_GetTick() > robomaster->feedback.lastFeedbackTime + robomaster->config.feedbackTimeout) {
		RoboMaster_reset(robomaster);
		return;
	}

	float minCurrent = robomaster->config.pidVelocity.config.minOut;
	float maxCurrent = robomaster->config.pidVelocity.config.maxOut;
	switch (robomaster->config.model) {
	case ROBOMASTER_M2006:
		minCurrent = MAX(minCurrent, -ROBOMASTER_M2006_MAX_CURRENT);
		maxCurrent = MIN(maxCurrent, ROBOMASTER_M2006_MAX_CURRENT);
		break;
	case ROBOMASTER_M3508:
		minCurrent = MAX(minCurrent, -ROBOMASTER_M3508_MAX_CURRENT);
		maxCurrent = MIN(maxCurrent, ROBOMASTER_M3508_MAX_CURRENT);
		break;
	default:
		RoboMaster_stop(robomaster);
		return;
	}

	float outputCurrent;
	float targetVelocity;
	float error;
	switch (robomaster->mode) {
	case ROBOMASTER_MODE_CURRENT:
		outputCurrent = robomaster->target;
		break;
	case ROBOMASTER_MODE_VELOCITY:
		outputCurrent = PID_calc(&robomaster->config.pidVelocity, robomaster->target, robomaster->feedback.velocity);
		break;
	case ROBOMASTER_MODE_POSITION:
		targetVelocity = PID_calc(&robomaster->config.pidPosition, robomaster->target, robomaster->feedback.position);
		outputCurrent = PID_calc(&robomaster->config.pidVelocity, targetVelocity, robomaster->feedback.velocity);
		break;
	case ROBOMASTER_MODE_TRAPEZOID:
		if (robomaster->config.trapezoidDeadband < 0.0f)
			robomaster->config.trapezoidDeadband = 0.0f;
		if (robomaster->config.trapezoidMaxAcceleration < 0.0f)
			robomaster->config.trapezoidMaxAcceleration = 0.0f;
		if (robomaster->config.trapezoidAcceleration > robomaster->config.trapezoidMaxAcceleration)
			robomaster->config.trapezoidAcceleration = robomaster->config.trapezoidMaxAcceleration;
		error = robomaster->target - robomaster->feedback.position;
if (error >= 0.0f) {
			error = MAX(0.0f, error - robomaster->config.trapezoidDeadband);
			targetVelocity = MIN(robomaster->config.pidVelocity.target + robomaster->config.trapezoidAcceleration * robomaster->config.pidVelocity.config.dt,
					MAX(robomaster->config.pidVelocity.target - robomaster->config.trapezoidMaxAcceleration * robomaster->config.pidVelocity.config.dt,
							sqrtf(2.0f * robomaster->config.trapezoidAcceleration * error)));
} else {
			error = MIN(0.0f, error + robomaster->config.trapezoidDeadband);
			targetVelocity = MAX(robomaster->config.pidVelocity.target - robomaster->config.trapezoidAcceleration * robomaster->config.pidVelocity.config.dt,
					MIN(robomaster->config.pidVelocity.target + robomaster->config.trapezoidMaxAcceleration * robomaster->config.pidVelocity.config.dt,
							-sqrtf(-2.0f * robomaster->config.trapezoidAcceleration * error)));
		}
		outputCurrent = PID_calc(&robomaster->config.pidVelocity, targetVelocity, robomaster->feedback.velocity);
		break;
	default:
		RoboMaster_stop(robomaster);
		return;
	}

	outputCurrent = MIN(MAX(outputCurrent, minCurrent), maxCurrent);

	robomaster->outputCurrent = outputCurrent;
}

void RoboMaster_transmit(RoboMasters *robomasters) {
	void (*canTxFunc)(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) = robomasters->canTxFunc;
	if (canTxFunc == NULL) return;
	uint8_t txData[8];
	int16_t rawCurrents[8];
	for (uint8_t i = 0; i < 8; i++) {
		RoboMaster *robomaster = &robomasters->robomaster[i];
		switch (robomaster->config.model) {
		case ROBOMASTER_M2006:
			rawCurrents[i] = (int16_t)(robomaster->outputCurrent * (ROBOMASTER_M2006_MAX_RAW_CURRENT / ROBOMASTER_M2006_MAX_CURRENT));
			rawCurrents[i] = MIN(MAX(rawCurrents[i], -ROBOMASTER_M2006_MAX_RAW_CURRENT), ROBOMASTER_M2006_MAX_RAW_CURRENT);
			break;
		case ROBOMASTER_M3508:
			rawCurrents[i] = (int16_t)(robomaster->outputCurrent * (ROBOMASTER_M3508_MAX_RAW_CURRENT / ROBOMASTER_M3508_MAX_CURRENT));
			rawCurrents[i] = MIN(MAX(rawCurrents[i], -ROBOMASTER_M3508_MAX_RAW_CURRENT), ROBOMASTER_M3508_MAX_RAW_CURRENT);
			break;
		default:
			rawCurrents[i] = 0;
		}
	}

	for (uint8_t i = 0; i < 4; i++) {
		txData[i * 2] = (uint8_t)((rawCurrents[i] >> 8) & 0xFF);
		txData[i * 2 + 1] = (uint8_t)(rawCurrents[i] & 0xFF);
	}
	canTxFunc(0x200, txData, sizeof(txData), CAN_ID_STD, CAN_RTR_DATA);

	for (uint8_t i = 0; i < 4; i++) {
		txData[i * 2] = (uint8_t)((rawCurrents[i + 4] >> 8) & 0xFF);
		txData[i * 2 + 1] = (uint8_t)(rawCurrents[i + 4] & 0xFF);
	}
	canTxFunc(0x1FF, txData, sizeof(txData), CAN_ID_STD, CAN_RTR_DATA);
}

bool RoboMaster_rxTask(RoboMasters *robomasters, uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (isExtended || isRemote || dlc != 8) return false;
	if (id < 0x201 || id > 0x208) return false;

	RoboMaster *robomaster = &robomasters->robomaster[id - 0x201];
	robomaster->feedback.lastRawPosition = robomaster->feedback.rawPosition;
	robomaster->feedback.rawPosition = (data[0] << 8) | data[1];
	robomaster->feedback.rawVelocity = (data[2] << 8) | data[3];
	robomaster->feedback.rawCurrent = (data[4] << 8) | data[5];
	robomaster->feedback.lastFeedbackTime = HAL_GetTick();
	if (robomaster->feedback.rawPosition - robomaster->feedback.lastRawPosition > ROBOMASTER_ENCODER_RESOLUTION / 2) {
		robomaster->feedback.roundCount--;
	}
	else if (robomaster->feedback.rawPosition - robomaster->feedback.lastRawPosition < -ROBOMASTER_ENCODER_RESOLUTION / 2) {
		robomaster->feedback.roundCount++;
	}
	robomaster->feedback.position = (robomaster->feedback.roundCount * ROBOMASTER_ENCODER_RESOLUTION + robomaster->feedback.rawPosition - robomaster->config.rawPositionOffset) * (2.0f * PI / ROBOMASTER_ENCODER_RESOLUTION);
	robomaster->feedback.velocity = robomaster->feedback.rawVelocity * (2.0f * PI / 60.0f);
	switch (robomaster->config.model) {
	case ROBOMASTER_M2006:
		robomaster->feedback.current = robomaster->feedback.rawCurrent * (ROBOMASTER_M2006_MAX_CURRENT / ROBOMASTER_M2006_MAX_RAW_CURRENT);
		break;
	case ROBOMASTER_M3508:
		robomaster->feedback.current = robomaster->feedback.rawCurrent * (ROBOMASTER_M3508_MAX_CURRENT / ROBOMASTER_M3508_MAX_RAW_CURRENT);
		break;
	default:
		RoboMaster_reset(robomaster);
	}
	return true;
}
