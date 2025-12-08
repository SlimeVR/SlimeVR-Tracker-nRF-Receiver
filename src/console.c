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
#include "parse_args.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>
#include "connection/esb.h"

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

static inline void strtolower(char *str) {
	for(int i = 0; str[i]; i++) {
		str[i] = tolower(str[i]);
	}
}

static void print_help(void)
{
	printk("\nhelp                         Display this help text\n");

	printk("\ninfo                         Get device information\n");
	printk("uptime                       Get device uptime\n");
	printk("list                         Get paired devices\n");
	printk("reboot                       Soft reset the device\n");
	printk("\nadd <address>                Manually add a device\n");
	printk("remove                       Remove last device\n");
	printk("pair                         Enter pairing mode\n");
	printk("exit                         Exit pairing mode\n");
	printk("clear                        Clear stored devices\n");
#if DFU_EXISTS
	printk("\ndfu                          Enter DFU bootloader\n");
#endif
	printk("\nmeow                         Meow!\n");
}

static void console_thread(void)
{
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	print_help();

	const char command_help[] = "help";

	const char command_info[] = "info";
	const char command_uptime[] = "uptime";
	const char command_list[] = "list";
	const char command_reboot[] = "reboot";
	const char command_add[] = "add";
	const char command_remove[] = "remove";
	const char command_pair[] = "pair";
	const char command_exit[] = "exit";
	const char command_clear[] = "clear";
#if DFU_EXISTS
	const char command_dfu[] = "dfu";
#endif
	const char command_meow[] = "meow";

	while (1) {
		char *line = console_getline();

		char* argv[5] = {NULL}; // command and 4 args
		size_t argc = parse_args(line, argv, ARRAY_SIZE(argv));
		if(argc == 0)
			continue;
		if(argc > 0)
			strtolower(argv[0]); // lower case the command
		if(argc > 1)
			strtolower(argv[1]); // lower case the first argument
		// only care that the first words are matchable

		if (strcmp(argv[0], command_help) == 0)
		{
			print_help();
		}
		else if (strcmp(argv[0], command_info) == 0)
		{
			print_info();
		}
		else if (strcmp(line, command_uptime) == 0)
		{
			print_uptime();
		}
		else if (strcmp(line, command_add) == 0)
		{
			if (argc != 2)
			{
				printk("Invalid number of arguments\n");
				continue;
			}
			uint64_t addr = parse_u64(argv[1], 16);
			uint8_t buf[13];
			snprintk(buf, 13, "%012llx", addr);
			if (addr != 0 && strcmp(buf, argv[1]) == 0)
				esb_add_pair(addr, true);
			else
				printk("Invalid address\n");
		}
		else if (strcmp(line, command_remove) == 0)
		{
			esb_pop_pair();
		}
		else if (strcmp(line, command_list) == 0)
		{
			print_list();
		}
		else if (strcmp(line, command_reboot) == 0)
		{
			skip_dfu();
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (strcmp(line, command_pair) == 0)
		{
			esb_reset_pair();
		}
		else if (strcmp(line, command_exit) == 0)
		{
			esb_finish_pair();
		}
		else if (strcmp(line, command_clear) == 0)
		{
			esb_clear();
		}
#if DFU_EXISTS
		else if (strcmp(line, command_dfu) == 0)
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
		else if (strcmp(line, command_meow) == 0)
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