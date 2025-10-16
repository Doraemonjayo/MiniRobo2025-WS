/*
 * PID.h
 *
 *  Created on: May 5, 2025
 *      Author: Doraemonjayo
 */

#ifndef PID_H_
#define PID_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float kp;
	float ki;
	float kd;
	float dt;
	float minTarget;
	float maxTarget;
	float minOut;
	float maxOut;
	float minIntegral;
	float maxIntegral;
	float minError;
	float maxError;
	float minDeadband;
	float maxDeadband;
}PIDConfig;

typedef struct {
	PIDConfig config;
	float target;
	float feedback;
	float error;
	float integral;
	float pOut;
	float iOut;
	float dOut;
	float out;
} PID;

void PID_init(PID *pid, PIDConfig config);
void PID_setConfig(PID *pid, PIDConfig config);
void PID_reset(PID *pid);
float PID_calc(PID *pid, float target, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
