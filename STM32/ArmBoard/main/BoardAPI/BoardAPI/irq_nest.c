/*
 * irq_nest.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#include "BoardAPI.h"

static uint32_t irq_cnt = 0;

void disable_irq_nest() {
	if (irq_cnt == 0) __disable_irq();
	if (irq_cnt < UINT32_MAX) irq_cnt++;
}

void enable_irq_nest() {
	if (irq_cnt > 0) irq_cnt--;
	if (irq_cnt == 0) __enable_irq();
}
