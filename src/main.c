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
#include "system/system.h"
// #include "timer.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include "connection/esb.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2

int main(void) {
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);

	uint8_t reboot_counter = reboot_counter_read();

	uint8_t reset_mode = reboot_counter - 100;
	if (reboot_counter != 100) {
		reboot_counter_write(100);
	}

	clocks_start();

	switch (reset_mode) {
		case 1:
			LOG_INF("Pairing requested");
			sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
			for (int i = 0; i < stored_trackers; i++) {
				sys_read(
					STORED_ADDR_0 + i,
					&stored_tracker_addr[i],
					sizeof(stored_tracker_addr[0])
				);
			}
			esb_pair();  // this will not return
			break;
		case 2:
			sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
			LOG_INF("NVS Reset");
			LOG_INF("Pairing requested");
			esb_pair();  // this will not return
			break;
#if DFU_EXISTS  // Using Adafruit bootloader
		case 3:  // DFU_MAGIC_UF2_RESET, Reset mode DFU
			LOG_INF("DFU requested");
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
#endif
		default:
			sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
			if (stored_trackers == 0) {
				esb_pair();  // this will not return // TODO:
			}
			for (int i = 0; i < stored_trackers; i++) {
				sys_read(
					STORED_ADDR_0 + i,
					&stored_tracker_addr[i],
					sizeof(stored_tracker_addr[0])
				);
			}
			LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);
			break;
	}

	//	k_msleep(1); // TODO: fixes some weird issue with the device bootlooping, what
	//is the cause

	esb_receive();

	esb_initialize(false);

	esb_start_rx();

	// timer_init();

	return 0;
}