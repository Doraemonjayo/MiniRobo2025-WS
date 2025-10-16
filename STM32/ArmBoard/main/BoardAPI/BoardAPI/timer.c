/*
 * timer.c
 *
 *  Created on: May 19, 2025
 *      Author: Doraemonjayo
 */

#include "BoardAPI.h"

static uint32_t us_upper32 = 0;
static void (*task_1kHz)() = NULL;

void timer_startUs() {
	us_upper32 = 0;
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim5);
}

void timer_set1kHzTask(void (*task)()) {
	task_1kHz = task;
}

void timer_start1kHzTask() {
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	__HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim14);
}

void (*timer_get1kHzTask())() {
	return task_1kHz;
}

uint32_t timer_getUs() {
	return htim5.Instance->CNT;
}

uint64_t timer_getUs64() {
    uint32_t upper = us_upper32;
    uint32_t cnt = htim5.Instance->CNT;

    while (upper != us_upper32) {
        upper = us_upper32;
        cnt   = htim5.Instance->CNT;
    };

    return ((uint64_t)upper << 32) | cnt;
}

void timer_stopUs() {
	HAL_TIM_Base_Stop_IT(&htim5);
	us_upper32 = 0;
}

void timer_stop1kHzTask() {
	HAL_TIM_Base_Stop_IT(&htim14);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim5){
		us_upper32++;
	}else if(htim == &htim14){
		if(task_1kHz != NULL) task_1kHz();
	}
}
