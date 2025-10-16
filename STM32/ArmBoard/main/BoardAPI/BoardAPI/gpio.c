/*
 * gpio.c
 *
 *  Created on: Jun 8, 2025
 *      Author: Doraemonjayo
 */

#include "BoardAPI.h"

#define READ_PIN(name) (HAL_GPIO_ReadPin(name##_GPIO_Port, name##_Pin))
#define WRITE_PIN(name, state) (HAL_GPIO_WritePin(name##_GPIO_Port, name##_Pin, state))

static void (*usrBtnPressedCallback)() = NULL;
static void (*usrBtnReleasedCallback)() = NULL;

GPIO_PinState gpio_getUsbId() {
	return READ_PIN(USB_ID);
}

GPIO_PinState gpio_getUsrBtn() {
	return READ_PIN(USR_BTN);
}

void gpio_setLedR(GPIO_PinState state) {
	WRITE_PIN(LED_R, state);
}

void gpio_setLedG(GPIO_PinState state) {
	WRITE_PIN(LED_G, state);
}

void gpio_setLedB(GPIO_PinState state) {
	WRITE_PIN(LED_B, state);
}

void gpio_setUsrBtnPressedCallback(void (*callback)()) {
	usrBtnPressedCallback = callback;
}

void gpio_setUsrBtnReleasedCallback(void (*callback)()) {
	usrBtnReleasedCallback = callback;
}

void (*gpio_getUsrBtnPressedCallback())() {
	return usrBtnPressedCallback;
}

void (*gpio_getUsrBtnReleasedCallback())() {
	return usrBtnReleasedCallback;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USR_BTN_Pin) {
		if (READ_PIN(USR_BTN) == GPIO_PIN_RESET) {
			if (usrBtnPressedCallback != NULL) usrBtnPressedCallback();
		} else {
			if (usrBtnReleasedCallback != NULL) usrBtnReleasedCallback();
		}
	}
}
