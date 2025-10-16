/*
 * gpio.h
 *
 *  Created on: Jun 8, 2025
 *      Author: Doraemonjayo
 */

#ifndef BOARDAPI_BOARDAPI_GPIO_H_
#define BOARDAPI_BOARDAPI_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define USR_BTN_PRESSED GPIO_PIN_RESET
#define USR_BTN_RELEASED GPIO_PIN_SET

GPIO_PinState gpio_getUsbId();
GPIO_PinState gpio_getUsrBtn();

void gpio_setLedR(GPIO_PinState state);
void gpio_setLedG(GPIO_PinState state);
void gpio_setLedB(GPIO_PinState state);

void gpio_setUsrBtnPressedCallback(void (*callback)());
void gpio_setUsrBtnReleasedCallback(void (*callback)());

void (*gpio_getUsrBtnPressedCallback())();
void (*gpio_getUsrBtnReleasedCallback())();

#ifdef __cplusplus
}
#endif

#endif /* BOARDAPI_BOARDAPI_GPIO_H_ */
