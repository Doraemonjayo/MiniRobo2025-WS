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

#ifndef PI
#define PI ((float)M_PI)
#endif /* PI */

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#define MECANUM_BOARD

void setup();
void loop();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_H_ */
