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

static struct k_work report_send;

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

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1) // streaming reports
#define HID_EP_REPORT_COUNT 4

struct tracker_report ep_report_buffer[HID_EP_REPORT_COUNT];

LOG_MODULE_REGISTER(hid_event, LOG_LEVEL_INF);

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),
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
			epind++;
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

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
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

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
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
	usb_enabled = true;
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |proto   |batt    |batt_v  |temp    |brd_id  |mcu_id  |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |
//|4       |id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|255     |id      |addr                                                 |resv                                                                   |

#ifndef CONFIG_SOC_NRF52820
#include "util.h"
// last valid data
static float last_q_trackers[MAX_TRACKERS][4] = {0};
static uint32_t last_v_trackers[MAX_TRACKERS] = {0};
static uint8_t last_p_trackers[MAX_TRACKERS] = {0};
static int last_valid_trackers[MAX_TRACKERS] = {0};
// last received data
static float cur_q_trackers[MAX_TRACKERS][4] = {0};
static uint32_t cur_v_trackers[MAX_TRACKERS] = {0};
static uint8_t cur_p_trackers[MAX_TRACKERS] = {0};
#endif

void hid_write_packet_n(uint8_t *data, uint8_t rssi)
{
#ifndef CONFIG_SOC_NRF52820
	// discard packets with abnormal rotation // TODO:
	if (data[0] == 1 || data[0] == 2 || data[0] == 4)
	{
		float v[3] = {0};
		float q[4] = {0};
		int16_t *buf = (int16_t *)&data[2];
		uint32_t *q_buf = (uint32_t *)&data[5];
		if (data[0] == 1 || data[0] == 4)
		{
			q[0] = FIXED_15_TO_DOUBLE(buf[3]);
			q[1] = FIXED_15_TO_DOUBLE(buf[0]);
			q[2] = FIXED_15_TO_DOUBLE(buf[1]);
			q[3] = FIXED_15_TO_DOUBLE(buf[2]);
			float mag = q_mag(q);
			if (mag < 0.999f || mag > 1.001f) // make sure the quaternion is valid
			{
				LOG_ERR("Abnormal quat %012llX i%d p%d n%.2f", stored_tracker_addr[data[1]], data[1], data[0], (double)mag);
				printk("a: %5.2f %5.2f %5.2f %5.2f\n", (double)q[0], (double)q[1], (double)q[2], (double)q[3]);
				return;
			}
		}
		else
		{
			v[0] = FIXED_10_TO_DOUBLE(*q_buf & 1023);
			v[1] = FIXED_11_TO_DOUBLE((*q_buf >> 10) & 2047);
			v[2] = FIXED_11_TO_DOUBLE((*q_buf >> 21) & 2047);
			for (int i = 0; i < 3; i++)
				v[i] = v[i] * 2 - 1;
			q_iem(v, q);
		}
		float zero[4] = {0};
		float *last_q = last_q_trackers[data[1]];
		bool last_invalid = !memcmp(last_q, zero, sizeof(zero));
		uint32_t *last_v = &last_v_trackers[data[1]];
		uint8_t *last_p = &last_p_trackers[data[1]];
		float *cur_q = cur_q_trackers[data[1]];
		uint32_t *cur_v = &cur_v_trackers[data[1]];
		uint8_t *cur_p = &cur_p_trackers[data[1]];
		float mag = q_diff_mag(q, last_q); // difference between last valid
		float mag_cur = q_diff_mag(q, cur_q); // difference between last received
		bool mag_invalid = mag > ROTATION_THRESHOLD && mag < 6.28f - ROTATION_THRESHOLD; // possibly invalid rotation
		bool mag_cur_invalid = mag_cur > ROTATION_THRESHOLD && mag_cur < 6.28f - ROTATION_THRESHOLD; // possibly inconsistent rotation
		if (mag_invalid && !last_invalid)
		{
			if (!mag_cur_invalid && last_valid_trackers[data[1]] >= RESET_THRESHOLD - 1)
			{
				// HWID, ID, packet type, rotation difference (rad), last valid packet
				LOG_ERR("Abnormal rot. %012llX i%d p%d m%.2f/%.2f v%d", stored_tracker_addr[data[1]], data[1], data[0], (double)mag, (double)mag_cur, last_valid_trackers[data[1]]);
				// decoded quat, packet type, q_buf
				printk("a: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)q[0], (double)q[1], (double)q[2], (double)q[3], data[0], *q_buf);
				printk("b: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)cur_q[0], (double)cur_q[1], (double)cur_q[2], (double)cur_q[3], *cur_p, *cur_v);
				printk("c: %5.2f %5.2f %5.2f %5.2f p%d:%08X\n", (double)last_q[0], (double)last_q[1], (double)last_q[2], (double)last_q[3], *last_p, *last_v);
			}
			last_valid_trackers[data[1]]++;
			memcpy(cur_q, q, sizeof(q));
			*cur_v = *q_buf;
			*cur_p = data[0];
			if (!mag_cur_invalid && last_valid_trackers[data[1]] >= RESET_THRESHOLD) // reset last_q
			{
				LOG_WRN("Reset rotation for %012llX, ID %d", stored_tracker_addr[data[1]], data[1]);
				last_valid_trackers[data[1]] = 0;
				memcpy(last_q, q, sizeof(q));
				*last_v = *q_buf;
				*last_p = data[0];
				return;
			}
			return;
		}
		last_valid_trackers[data[1]] = 0;
		memcpy(cur_q, q, sizeof(q));
		*cur_v = *q_buf;
		*cur_p = data[0];
		memcpy(last_q, q, sizeof(q));
		*last_v = *q_buf;
		*last_p = data[0];
	}
#endif

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
