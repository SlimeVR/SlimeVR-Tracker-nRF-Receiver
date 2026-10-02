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
#include "connection/esb.h"
#include "connection/messages.h"

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

static struct k_work report_send;
static struct k_work report_read;

static struct report_t {
	uint8_t length;
	uint8_t data[64];
};

struct k_msgq reports;
K_MSGQ_DEFINE(reports, sizeof(struct report_t), MAX_TRACKERS, 1);

static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);
static ATOMIC_DEFINE(hid_ep_out_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1) // streaming reports // TODO: could it be shorter/reduce latency?
#define POLL_PERIOD		K_MSEC(1) // streaming reports // TODO: could it be shorter/reduce latency?
#define HID_EP_REPORT_COUNT 4

uint8_t report_buffer[64];
uint8_t ep_read_buffer[256]; // TODO: no struct // TODO: is possible to read >64 bytes, e.g. a delayed read?

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

static void report_read_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(read_timer, report_read_handler, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_OUTPUT(0x02),
	HID_END_COLLECTION,
};

uint16_t sent_device_addr = 0;
bool usb_enabled = false;
int64_t last_registration_sent = 0;

static void packet_device_addr(uint8_t *report, uint16_t id) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = id;
	memcpy(&report[2], &stored_tracker_addr[id], 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

static void send_report(struct k_work *work)
{
	if (!usb_enabled) return;
	if (!stored_trackers) return;

	bool have_reports = k_msgq_num_used_get(&reports) > 0;

	if (!have_reports && k_uptime_get() - 100 < last_registration_sent) {
		return; // send registrations only every 100ms
	}

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		last_registration_sent = k_uptime_get();
		int ret, wrote;
		int buffer_index = 0;
		struct report_t report;
		while(true) {
			int ret = k_msgq_peek(&reports, &report);
			if(ret < 0)
				break;
			int buffer_remainder = sizeof(report_buffer) - buffer_index;
			if(buffer_remainder >= report.length) {
				ret = k_msgq_get(&reports, &report, K_NO_WAIT);
				if(ret < 0)
					break;
				memcpy(report_buffer, &report.data, report.length);
				buffer_index += report.length;
			} else {
				break;
			}
		}
		// Pad remaining report slots with device addr
		// TODO : Not necessarry on Protocol 3
		// TODO : Pad with RSSI packets
		while(sizeof(report_buffer) - buffer_index >= 16) {
			if (stored_trackers > 0) {
				packet_device_addr(&report_buffer[buffer_index], sent_device_addr);
				sent_device_addr = (sent_device_addr + 1) % stored_trackers;
				buffer_index += 16;
			}
		}

		ret = hid_int_ep_write(hdev, report_buffer, sizeof(report_buffer), &wrote);

		if (ret != 0) {
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			//LOG_DBG("Report submitted");
		}
	} else { // busy with what
		//LOG_DBG("HID IN endpoint busy");
	}
}

void hid_report_received(uint8_t * report_buffer, int length) {
	
	for (int offset = 0; offset < length; offset += 16)
	{
		uint8_t *packet = report_buffer + offset;
		uint8_t packet_id = packet[1];
		if(packet_id == 0)
			continue;
		// TODO Lengths???
		// Message is for tracker
		if ((packet_id >= 1) & (packet_id <= 200))
		{
			esb_tracker_message(packet, 16);
		}
		else
		{
			hid_dongle_message(packet, 16);
		}
	}
}

static void read_report(struct k_work *work)
{
	if (!usb_enabled) return;

	int ret, read;

	if (!atomic_test_and_set_bit(hid_ep_out_busy, HID_EP_BUSY_FLAG)) {
		ret = hid_int_ep_read(hdev, (uint8_t *)ep_read_buffer, sizeof(ep_read_buffer), &read);

		if (ret != 0) {
			LOG_ERR("hid_int_ep_read: %d", ret);
		} else {
			LOG_INF("hid_int_ep_read: %d", read);
			hid_report_received(ep_read_buffer, read);
		}
	} else { // busy with what
		//LOG_DBG("HID OUT endpoint busy");
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
	// TODO: can probably immediately write report from here
}

void hid_int_in_ready(void)
{
	int_in_ready_cb(hdev);
}

static void int_out_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_out_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("OUT endpoint callback without preceding buffer write");
	}
	// TODO: can probably immediately read report from here
}

/*
 * On Idle callback is available here as an example even if actual use is
 * very limited. In contrast to report_event_handler(),
 * report value is not incremented here.
 */
static void on_idle_cb(const struct device *dev, uint16_t report_id)
{
	LOG_DBG("On idle callback");
	k_work_submit(&report_send);
}

static void report_event_handler(struct k_timer *dummy)
{
	if (usb_enabled)
		k_work_submit(&report_send);
}

static void report_read_handler(struct k_timer *dummy)
{
	if (usb_enabled)
		k_work_submit(&report_read);
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
	.int_out_ready = int_out_ready_cb,
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
};

static int composite_pre_init()
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&event_timer, REPORT_PERIOD, REPORT_PERIOD);

	atomic_set_bit(hid_ep_out_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&read_timer, POLL_PERIOD, POLL_PERIOD);

	if (usb_hid_set_proto_code(hdev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code");
	}

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void hid_init(void)
{
	k_work_init(&report_send, send_report);
	k_work_init(&report_read, read_report);
	usb_enabled = true;
}

void hid_write_packet_n(uint8_t *data, size_t size)
{
	struct report_t report;
	report.length = size;
	memcpy(&report.data, data, size);
	
	k_msgq_put(&reports, &report, K_NO_WAIT);
}
