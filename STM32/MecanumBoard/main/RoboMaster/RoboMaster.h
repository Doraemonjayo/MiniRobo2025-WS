/*
 * RoboMaster.h
 *
 *  Created on: Jun 14, 2025
 *      Author: Doraemonjayo
 */

#ifndef ROBOMASTER_ROBOMASTER_H_
#define ROBOMASTER_ROBOMASTER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "BoardAPI.h"
#include "PID.h"

#define ROBOMASTER_M2006_MAX_CURRENT 10.0f // [A]
#define ROBOMASTER_M3508_MAX_CURRENT 20.0f // [A]
#define ROBOMASTER_M2006_MAX_RAW_CURRENT 10000
#define ROBOMASTER_M3508_MAX_RAW_CURRENT 16384

#define ROBOMASTER_DATALEN 8
#define ROBOMASTER_ENCODER_RESOLUTION 8192

typedef enum {
	ROBOMASTER_M2006,
	ROBOMASTER_M3508,
} RoboMasterModel;

typedef enum {
	ROBOMASTER_MODE_CURRENT,
	ROBOMASTER_MODE_VELOCITY,
	ROBOMASTER_MODE_POSITION,
	ROBOMASTER_MODE_TRAPEZOID,
} RoboMasterMode;

typedef struct {
	RoboMasterModel model;
	PID pidVelocity;
	PID pidPosition;
	uint32_t controlTimeout;   // [ms]
	uint32_t feedbackTimeout;  // [ms]
	int16_t rawPositionOffset; // [1/8192]
	float trapezoidAcceleration;
	float trapezoidMaxAcceleration;
	float trapezoidDeadband;
} RoboMasterConfig;

typedef struct {
	int16_t rawCurrent;
	int16_t rawVelocity;	 // [rpm]
	int16_t rawPosition;	 // [1/8192]
	int16_t lastRawPosition; // [1/8192]
	int32_t roundCount;		 // [rounds]

	float current;	// [A]
	float velocity; // [rad/s]
	float position; // [rad]

	uint32_t lastFeedbackTime;		  // [ms]
	uint32_t lastControlTime;		  // [ms]
} RoboMasterFeedback;

struct RoboMasters;

typedef struct {
	RoboMasterConfig config;
	RoboMasterFeedback feedback;
	RoboMasterMode mode;
	float target;
	float outputCurrent; // [A]
} RoboMaster;

typedef struct RoboMasters {
	RoboMaster robomaster[8];
	void (*canTxFunc)(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
} RoboMasters;

extern const PIDConfig ROBOMASTER_M2006_PID_VELOCITY_DEFAULT;
extern const PIDConfig ROBOMASTER_M2006_PID_POSITION_DEFAULT;
extern const PIDConfig ROBOMASTER_M3508_PID_VELOCITY_DEFAULT;
extern const PIDConfig ROBOMASTER_M3508_PID_POSITION_DEFAULT;

void RoboMaster_resetFeedback(RoboMaster *robomaster);
void RoboMaster_stop(RoboMaster *robomaster);
void RoboMaster_reset(RoboMaster *robomaster);
void RoboMaster_init(RoboMaster *robomaster, RoboMasterConfig config);
void RoboMasters_setTxFunc(RoboMasters *robomasters, void (*canTxFunc)(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote));

void RoboMaster_setModel(RoboMaster *robomaster, RoboMasterModel model);
void RoboMaster_setMode(RoboMaster *robomaster, RoboMasterMode mode);
void RoboMaster_setControlTimeout(RoboMaster *robomaster, uint32_t timeout);
void RoboMaster_setFeedbackTimeout(RoboMaster *robomaster, uint32_t timeout);
void RoboMaster_setTimeout(RoboMaster *robomaster, uint32_t controlTimeout, uint32_t feedbackTimeout);
void RoboMaster_setCurrentLimit(RoboMaster *robomaster, float minCurrent, float maxCurrent);
void RoboMaster_setVelocityLimit(RoboMaster *robomaster, float minVelocity, float maxVelocity);
void RoboMaster_setPositionLimit(RoboMaster *robomaster, float minPosition, float maxPosition);
void RoboMaster_setTrapezoidAcceleration(RoboMaster *robomaster, float acceleration);
void RoboMaster_setTrapezoidMaxAcceleration(RoboMaster *robomaster, float acceleration);
void RoboMaster_setRoundCount(RoboMaster *robomaster, int32_t roundCount);
void RoboMaster_setRawPositionOffset(RoboMaster *robomaster, int16_t offset);
void RoboMaster_resetZeroPosition(RoboMaster *robomaster);

void RoboMaster_setTargetCurrent(RoboMaster *robomaster, float current);
void RoboMaster_setTargetVelocity(RoboMaster *robomaster, float velocity);
void RoboMaster_setTargetPosition(RoboMaster *robomaster, float position);
void RoboMaster_setTargetTrapezoid(RoboMaster *robomaster, float position);

void RoboMaster_calculateOutputCurrent(RoboMaster *robomaster);
void RoboMaster_transmit(RoboMasters *robomasters);

bool RoboMaster_rxTask(RoboMasters *robomasters, uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);

#ifdef __cplusplus
}
#endif

#endif /* ROBOMASTER_ROBOMASTER_H_ */
