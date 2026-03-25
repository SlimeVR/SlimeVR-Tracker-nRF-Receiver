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
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/sys/crc.h>

static struct k_work report_send;
static struct k_work report_read;

static struct tracker_report {
	uint8_t data[16];
} __packed report = {
	.data = {0}
};;

struct tracker_report reports[MAX_TRACKERS];
atomic_t report_write_index = 0;
atomic_t report_read_index = 0;
// read_index == write_index -> empty fifo
// (write_index + 1) % MAX_TRACKERS == read_index -> full fifo

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);
static ATOMIC_DEFINE(hid_ep_out_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1) // streaming reports // TODO: could it be shorter/reduce latency?
#define POLL_PERIOD		K_MSEC(1) // streaming reports // TODO: could it be shorter/reduce latency?
#define HID_EP_REPORT_COUNT 4

struct tracker_report ep_report_buffer[HID_EP_REPORT_COUNT];
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

//|type    |description
//|TX   255|receiver packet 0, associate id and tracker address
//|TX   254|device list hash, round trip ping
//|TX   253|receiver status (TODO: send number of pairing requests found?)
//|TX   252|device pairing request, id is the id of the pairing request in case of multiple, not final tracker id (TODO: these can timeout if not sent often? can server automatically accept if the address was known?)
//|RX   255|round trip response
//|RX   254|command (TODO: include parameters?)

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |data                                                                                                                                  |
//|TX   255|id      |device_addr                                          |resv-------------------------------------------------------------------|
//|TX   254|stored  |crc                                |latency          |resv-------------------------------------------------------------------|
//|TX   253|status  |resv-------------------------------------------------|resv-------------------------------------------------------------------|
//|TX   252|id      |device_addr                                          |resv-------------------------------------------------------------------|
//|RX   255|resv----------------------------------------------------------|resv-------------------------------------------------------------------|
//|RX   254|command |resv-------------------------------------------------|resv-------------------------------------------------------------------|

// TODO: implement this

static void packet_device_addr(uint8_t *report, uint16_t id) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = id;
	memcpy(&report[2], &stored_tracker_addr[id], 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

static void packet_device_list_hash(uint8_t *report, uint16_t id) // associate id and tracker address
{
	// note: stored addresses use 6 bytes for the address, last two bytes are zeroes
	uint32_t crc = crc32_k_4_2_update(0x93a409eb, stored_tracker_addr, (uint16_t)stored_trackers * 8);
	report[0] = 0xAA; // magic :D
	report[1] = 0xAA; // magic :D
	report[2] = 0xAA; // magic :D
	report[3] = 0xAA; // magic :D
	*(uint16_t *)report[4] = LATENCY_US; // default to zero if not known
	report[6] = 0; // unused
	report[7] = stored_trackers;
	memcpy(&report[8], &crc, 8);
}

static uint32_t dropped_reports = 0;
static uint16_t max_dropped_reports = 0;

static void send_report(struct k_work *work)
{
	if (!usb_enabled) return;
	if (!stored_trackers) return;

	// Get current FIFO status atomically
	size_t write_idx = (size_t)atomic_get(&report_write_index);
	size_t read_idx = (size_t)atomic_get(&report_read_index);

	if (write_idx == read_idx && k_uptime_get() - 100 < last_registration_sent) {
		return; // send registrations only every 100ms
	}

	int ret, wrote;

	last_registration_sent = k_uptime_get();

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		// Calculate how many reports we have available
		int available_reports = write_idx - read_idx;
		if (available_reports < 0) available_reports += MAX_TRACKERS;
		size_t reports_to_send = (size_t)((available_reports > HID_EP_REPORT_COUNT) ? HID_EP_REPORT_COUNT : available_reports);

		int epind;
		// Copy existing data to buffer
		for (epind = 0; epind < reports_to_send; epind++) {
			ep_report_buffer[epind] = reports[read_idx];
			read_idx++;
			if (read_idx == MAX_TRACKERS) read_idx = 0;
			atomic_set(&report_read_index, read_idx);
		}

		// Pad remaining report slots with device addr
		for (; epind < HID_EP_REPORT_COUNT; epind++) {
			if (stored_trackers > 0) {
				packet_device_addr(ep_report_buffer[epind].data, sent_device_addr);
				sent_device_addr = (sent_device_addr + 1) % stored_trackers;
			}
		}

		ret = hid_int_ep_write(hdev, (uint8_t *)ep_report_buffer, sizeof(report) * HID_EP_REPORT_COUNT, &wrote);

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

#define DROPPED_REPORT_LOG_INTERVAL 5000

static void hid_dropped_reports_logging(void)
{
	while (1) {
		if (dropped_reports) LOG_INF("Dropped reports: %u (max: %u)", dropped_reports, max_dropped_reports);
		dropped_reports = 0;
		max_dropped_reports = 0;
		k_msleep(DROPPED_REPORT_LOG_INTERVAL);
	}
}

K_THREAD_DEFINE(hid_dropped_reports_logging_thread, 256, hid_dropped_reports_logging, NULL, NULL, NULL, 6, 0, 0);

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
			// do something here
			LOG_INF("%016llX%016llX%016llX%016llX%016llX%016llX%016llX%016llX",
				*(uint64_t *)(ep_read_buffer + 56),
				*(uint64_t *)(ep_read_buffer + 48),
				*(uint64_t *)(ep_read_buffer + 40),
				*(uint64_t *)(ep_read_buffer + 32),
				*(uint64_t *)(ep_read_buffer + 24),
				*(uint64_t *)(ep_read_buffer + 16),
				*(uint64_t *)(ep_read_buffer + 8),
				*(uint64_t *)ep_read_buffer
			);
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

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		int configurationIndex = *param;
		if(configurationIndex == 0) {
			// from usb_device.c: A configuration index of 0 unconfigures the device.
			configured = false;
		} else {
			if (!configured) {
				int_in_ready_cb(hdev);
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

void usb_init_thread(void)
{
	usb_enable(status_cb);
	k_work_init(&report_send, send_report);
	k_work_init(&report_read, read_report);
	usb_enabled = true;
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

//|type    |description
//|RX     0|device info ("info")
//|RX     1|full precision quat and accel
//|RX     2|reduced precision quat and accel with battery, temp, and rssi ("info")
//|RX     3|status ("status")
//|RX     4|full precision quat and magnetometer
//|RX     5|runtime ("status2")
//|RX     6|reduced precision quat and accel with button and sleep time ("info2")
//|RX     7|button and sleep time ("info2")

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|RX     0|id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv----|imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|RX     1|id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|RX     2|id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|RX     3|id      |svr_stat|status  |resv----------------------------------------------------------------------------------------------|rssi    |
//|RX     4|id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|RX     5|id      |runtime                                                                |resv----------------------------------------|rssi    |
//|RX     6|id      |button  |sleeptime        |resv-------------------------------------------------------------------------------------|rssi    |
//|RX     7|id      |button  |sleeptime        |q_buf                              |a0               |a1               |a2               |rssi    |

// runtime is in microseconds (overkill), sleeptime is in milliseconds (overkill but less)

void hid_write_packet_n(uint8_t *data, uint8_t rssi)
{
	memcpy(&report.data, data, sizeof(report)); // all data can be passed through
	if (data[0] != 1 && data[0] != 4) // packet 1 and 4 are full precision quat and accel/mag, no room for rssi
		report.data[15] = rssi;
	// Get current FIFO status atomically
	size_t write_idx = (size_t)atomic_get(&report_write_index);
	size_t read_idx = (size_t)atomic_get(&report_read_index);

	// Try to replace existing entry for the same tracker first
	if (write_idx != read_idx) {
		// Start from read point + 1 to avoid hitting the entry being used
		size_t check_index = read_idx + 1;
		if (check_index == MAX_TRACKERS) check_index = 0;

		while (check_index != write_idx) {
			if (reports[check_index].data[1] == data[1]) {
				// Replace existing entry
				reports[check_index] = report;
				return;
			}
			check_index = check_index + 1;
			if (check_index == MAX_TRACKERS) check_index = 0;
		}
	}
	if (write_idx + 1 == read_idx || (write_idx == MAX_TRACKERS-1 && read_idx == 0)) { // overflow
		dropped_reports ++;
		return;
	}
	// Write new packet into FIFO
	reports[write_idx] = report;

	// Update write index atomically
	write_idx ++;
	if (write_idx == MAX_TRACKERS) write_idx = 0;
	atomic_set(&report_write_index, write_idx);
}
