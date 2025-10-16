/*
 * BoardAPI.h
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_H_
#define BOARDAPI_BOARDAPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"

#include "BoardAPI/cdc.h"
#include "BoardAPI/timer.h"
#include "BoardAPI/gpio.h"
#include "BoardAPI/can.h"
#include "BoardAPI/flash.h"
#include "BoardAPI/irq_nest.h"

#define MECANUM_BOARD

void setup();
void loop();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_H_ */
