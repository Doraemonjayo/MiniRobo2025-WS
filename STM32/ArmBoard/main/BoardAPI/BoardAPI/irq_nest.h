/*
 * irq_nest.h
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_IRQ_NEST_H_
#define BOARDAPI_BOARDAPI_IRQ_NEST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void disable_irq_nest();
void enable_irq_nest();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_IRQ_NEST_H_ */
