#include "globals.h"
#include "system.h"
//#include "timer.h"
#include "esb.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2

int main(void)
{
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);

#if DFU_EXISTS // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif

	uint8_t reboot_counter = reboot_counter_read();

	uint8_t reset_mode = reboot_counter - 100;
	if (reboot_counter != 100)
		reboot_counter_write(100);

	switch (reset_mode)
	{
	case 1:
		LOG_INF("Pairing requested");
		sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		esb_pair(); // this will not return
		break;
	case 2:
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		LOG_INF("NVS Reset");
		LOG_INF("Pairing requested");
		esb_pair(); // this will not return
		break;
#if DFU_EXISTS // Using Adafruit bootloader
	case 3: // DFU_MAGIC_UF2_RESET, Reset mode DFU
		LOG_INF("DFU requested");
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
#endif
	default:
		sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		if (stored_trackers == 0)
			esb_pair(); // this will not return // TODO:
		for (int i = 0; i < stored_trackers; i++)
			sys_read(STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
		LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);
		break;
	}

//	k_msleep(1); // TODO: fixes some weird issue with the device bootlooping, what is the cause

	clocks_start();

	esb_receive();

	esb_initialize(false);

	esb_start_rx();

	//timer_init();

	return 0;
}