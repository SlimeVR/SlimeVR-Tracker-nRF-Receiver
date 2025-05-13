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
#include "build_defines.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>
#include "esb.h"

#include <ctype.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
K_THREAD_DEFINE(console_thread_id, 1024, console_thread, NULL, NULL, NULL, 6, 0, 0);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

static const char *meows[] = {
	"Mew",
	"Meww",
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"mew",
	"meww",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static const char *meow_suffixes[] = {
	" :3",
	" :3c",
	" ;3",
	" ;3c",
	" x3",
	" x3c",
	" X3",
	" X3c",
	" >:3",
	" >:3c",
	" >;3",
	" >;3c",
	""
};

static void skip_dfu(void)
{
#if DFU_EXISTS // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif
}

static void print_info(void)
{
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
	printk(FW_STRING);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nDevice address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
}

static void print_uptime(void)
{
	int64_t uptime = k_ticks_to_us_floor64(k_uptime_ticks());

	uint32_t days = uptime / 86400000000;
	uptime %= 86400000000;
	uint8_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime %= 1000;

	printk("Uptime: %u.%02u:%02u:%02u.%03u,%03u\n", days, hours, minutes, seconds, milliseconds, microseconds);
}

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void print_list(void)
{
	printk("Stored devices:\n");
	for (uint8_t i = 0; i < stored_trackers; i++)
		printk("%012llX\n", stored_tracker_addr[i]);
}

static void console_thread(void)
{
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("info                         Get device information\n");
	printk("uptime                       Get device uptime\n");
	printk("list                         Get paired devices\n");
	printk("reboot                       Soft reset the device\n");
	printk("pair                         Enter pairing mode\n");
	printk("exit                         Exit pairing mode\n");
	printk("clear                        Clear stored devices\n");

	uint8_t command_info[] = "info";
	uint8_t command_uptime[] = "uptime";
	uint8_t command_list[] = "list";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_pair[] = "pair";
	uint8_t command_exit[] = "exit";
	uint8_t command_clear[] = "clear";

#if DFU_EXISTS
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_dfu[] = "dfu";
#endif

	printk("meow                         Meow!\n");

	uint8_t command_meow[] = "meow";

	while (1) {
		uint8_t *line = console_getline();
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
		}

		if (memcmp(line, command_info, sizeof(command_info)) == 0)
		{
			print_info();
		}
		else if (memcmp(line, command_uptime, sizeof(command_uptime)) == 0)
		{
			print_uptime();
		}
		else if (memcmp(line, command_list, sizeof(command_list)) == 0)
		{
			print_list();
		}
		else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
		{
			skip_dfu();
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0)
		{
			esb_reset_pair();
		}
		else if (memcmp(line, command_exit, sizeof(command_exit)) == 0)
		{
			esb_finish_pair();
		}
		else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) 
		{
			esb_clear();
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
#if ADAFRUIT_BOOTLOADER
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) 
		{
			print_meow();
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif