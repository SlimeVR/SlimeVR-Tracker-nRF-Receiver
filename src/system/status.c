/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"

#include <zephyr/kernel.h>

#include "status.h"
#include "led.h"

static int status_state = 0;

LOG_MODULE_REGISTER(status, LOG_LEVEL_INF);

static void status_thread(void);
K_THREAD_DEFINE(status_thread_id, 256, status_thread, NULL, NULL, NULL, 6, 0, 0);

void set_status(enum sys_status status, bool set) {
	if (set) {
		status_state |= status;
		switch (status) {
			case SYS_STATUS_SENSOR_ERROR:
				LOG_ERR("Sensor communication error");
				break;
			case SYS_STATUS_CONNECTION_ERROR:
				LOG_WRN("Connection error");
				break;
			case SYS_STATUS_SYSTEM_ERROR:
				LOG_ERR("General error");
				break;
			case SYS_STATUS_USB_CONNECTED:
				LOG_INF("USB connected");
				break;
			case SYS_STATUS_PLUGGED:
				LOG_INF("Charger plugged");
				break;
			default:
				break;
		}
	} else {
		status_state &= ~status;
		LOG_INF("Cleared status: %d", status);
	}
	LOG_INF("Status: %d", status_state);
}

static void status_thread(void) {
	while (1)  // cycle through errors
	{
		int status = status_state
				   & (SYS_STATUS_SENSOR_ERROR | SYS_STATUS_CONNECTION_ERROR
					  | SYS_STATUS_SYSTEM_ERROR);
		if (status & SYS_STATUS_SENSOR_ERROR) {
			set_led(SYS_LED_PATTERN_ERROR_A, SYS_LED_PRIORITY_STATUS);
			k_msleep(5000);
		}
		if (status & SYS_STATUS_CONNECTION_ERROR) {
			set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_STATUS);
			k_msleep(5000);
		}
		if (status & SYS_STATUS_SYSTEM_ERROR) {
			set_led(SYS_LED_PATTERN_ERROR_C, SYS_LED_PRIORITY_STATUS);
			k_msleep(5000);
		}
		if (!status) {
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_STATUS);
			k_msleep(100);
		}
	}
}

bool status_ready(void)  // true if no important statuses are active
{
	return (status_state & ~SYS_STATUS_CONNECTION_ERROR)
		== 0;  // connection error is temporary, not critical
}
