#include "globals.h"
#include "hid.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>

static bool configured;

LOG_MODULE_REGISTER(usb, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, USB_INIT_THREAD_PRIORITY, 0, 0);

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status)
	{
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		int configurationIndex = *param;
		if (configurationIndex == 0)
		{
			// from usb_device.c: A configuration index of 0 unconfigures the device.
			configured = false;
		}
		else
		{
			if (!configured)
			{
				hid_int_in_ready();
				configured = true;
			}
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static void usb_init_thread(void)
{
	usb_enable(status_cb);
	hid_init();
}
