/*
 * PID.c
 *
 *  Created on: May 5, 2025
 *      Author: Doraemonjayo
 */

#include "PID.h"
#include <math.h>

void PID_init(PID *pid, PIDConfig config){
	PID_setConfig(pid, config);
	PID_reset(pid);
}

void PID_setConfig(PID *pid, PIDConfig config){
	pid->config = config;
}

void PID_reset(PID *pid){
	pid->target = 0.0f;
	pid->feedback = 0.0f;
	pid->error = NAN;
	pid->integral = 0.0f;
	pid->pOut = 0.0f;
	pid->iOut = 0.0f;
	pid->dOut = 0.0f;
	pid->out = 0.0f;
}

float PID_calc(PID *pid, float target, float feedback){
	if(target > pid->config.maxTarget){
		target = pid->config.maxTarget;
	}else if(target < pid->config.minTarget){
		target = pid->config.minTarget;
	}
	float error = target - feedback;
	if(error > pid->config.maxDeadband){
		error -= pid->config.maxDeadband;
	}else if(error < pid->config.minDeadband){
		error -= pid->config.minDeadband;
	}else{
		error = 0.0f;
	}
	if(error > pid->config.maxError){
		error = pid->config.maxError;
	}else if(error < pid->config.minError){
		error = pid->config.minError;
	}
	float lastError = pid->error;
	pid->target = target;
	pid->feedback = feedback;
	pid->error = error;
	pid->integral += error * pid->config.dt;
	pid->pOut = pid->config.kp * error;
	pid->iOut = pid->config.ki * pid->integral;
	if(pid->config.ki == 0.0f){
		pid->integral = 0.0f;
		pid->iOut = 0.0f;
	}else if(pid->iOut > pid->config.maxIntegral){
		pid->integral = pid->config.maxIntegral / pid->config.ki;
		pid->iOut = pid->config.maxIntegral;
	}else if(pid->iOut < pid->config.minIntegral){
		pid->integral = pid->config.minIntegral / pid->config.ki;
		pid->iOut = pid->config.minIntegral;
	}
	if((!isnan(lastError)) && (pid->config.dt > 0.0f)){
		pid->dOut = pid->config.kd * (error - lastError) / pid->config.dt;
	}else{
		pid->dOut = 0.0f;
	}
	pid->out = pid->pOut + pid->iOut + pid->dOut;
	if(pid->out > pid->config.maxOut){
		pid->out = pid->config.maxOut;
	}else if(pid->out < pid->config.minOut){
		pid->out = pid->config.minOut;
	}
	return pid->out;
}
