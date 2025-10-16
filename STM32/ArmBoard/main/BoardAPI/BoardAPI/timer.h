/*
 * timer.h
 *
 *  Created on: May 19, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_TIMER_H_
#define BOARDAPI_BOARDAPI_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim14;

void timer_startUs();
void timer_set1kHzTask(void (*task)());
void timer_start1kHzTask();

void (*timer_get1kHzTask())();

uint32_t timer_getUs();
uint64_t timer_getUs64();

void timer_stopUs();
void timer_stop1kHzTask();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_TIMER_H_ */
