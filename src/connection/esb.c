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
#include "hid.h"
#include "tdma.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/crc.h>

#include "esb.h"

static struct esb_payload rx_payload;
static struct esb_payload tx_payload_dongle_sate = ESB_CREATE_PAYLOAD(0,
														0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0);
uint8_t sequences[255] = {0};
uint16_t packets_count[255] = {0};
uint8_t packets_lost[255] = {0};
uint64_t new_paired_address = 0;

uint32_t last_reset_time = 0;
uint32_t packets_received = 0;
bool test_packet_acked = false;

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 6, 0, 0);

int32_t tdma_timer_offset = 0;
int32_t tdma_timer_offset_static = -3;
uint8_t tracker_window = 0;
uint32_t packet_sent_time = 0;
uint8_t skip = 0;

struct con_stat {
	uint8_t packets_received;
	uint8_t packets_lost;
	uint8_t windows_hit;
	uint8_t windows_missed;
	uint8_t last_packet_number;
};

struct con_stat statistics[MAX_TRACKERS];

// Use this to generate ACK packet to send to tracker when we receive data from it
// Ideally, it should return as fast as possible
// Also must not execute any sys calls like sys_write and etc.
void ack_handler(uint8_t *pdu_data, uint8_t data_length, uint32_t pipe_id, struct esb_payload *ack_payload, bool *has_ack_payload) {
	// Handle new pairing requests
	if(data_length == 8 && pipe_id == 0 && new_paired_address == 0) {
		// TODO Implement proper pairing by packet header and such
		// make sure the packet is valid
		uint8_t checksum = crc8_ccitt(0x07, &pdu_data[2], 6);
		if (checksum == 0)
			checksum = 8;
		if (checksum != pdu_data[0]) {
			LOG_INF("Checksum error %d != %d", checksum, pdu_data[0]);
			return;
		}
		// Extract tracker's address
		uint64_t found_addr = (*(uint64_t *)&pdu_data[2]) & 0xFFFFFFFFFFFF;
		LOG_INF("Pairing request from %012llX", found_addr);
		uint16_t send_tracker_id = stored_trackers; // Use new tracker id
		for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
		{
			if (found_addr != 0 && stored_tracker_addr[i] == found_addr)
			{
				send_tracker_id = i;
			}
		}
		if (send_tracker_id >= MAX_TRACKERS) {
			LOG_INF("Too many registered trackers! %d >= %d", send_tracker_id, MAX_TRACKERS);
			return;
		}
		if (send_tracker_id == stored_trackers) // New device, add to NVS
		{
			// Save the address, we will save it in the esb thread
			new_paired_address = found_addr;
		}
		ack_payload->data[0] = checksum; // Use checksum sent from device to make sure packet is for that device
		ack_payload->data[1] = send_tracker_id; // Add tracker id to packet
		// Send our own address back
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		memcpy(&ack_payload->data[2], addr, 6);
		ack_payload->length = 8;
		*has_ack_payload = true;
	}
	if(pipe_id > 0 || pdu_data[0] == ESB_TEST_PREAMBLE) {
		// Check tracker's timing window and send its offset for TDMA
		uint32_t tdma_timer = tdma_get_timer();
		uint8_t tracker_window = 0;
		uint8_t tracker_id = WRONG_TRACKER_ID;
		if(pipe_id > 0) {
			// Tracker ID is always at byte 1
			tracker_id = pdu_data[1];
			if(tracker_id == WRONG_TRACKER_ID) {
				// Not tracker packet, skip
				return;
			}
			tracker_window = tdma_get_or_allocate_tracker_window(tracker_id);
			if(tracker_window == TDMA_WRONG_WINDOW) {
				// No windows left for this tracker, reject it
				ack_payload->data[0] = ESB_CONTROL_PREAMBLE;
				ack_payload->data[1] = ESB_PACKET_CONTROL_NO_WINDOWS; // No Windows (4)
				ack_payload->length = 2;
				*has_ack_payload = true;
				return;
			}
		}
		uint32_t current_slot = tdma_get_slot(tdma_timer);
		uint8_t current_window = tdma_get_window(current_slot);
		if(!tdma_is_dongle_window(current_slot) && current_window == tracker_window) {
			// Tracker sent data at the correct time, we can send data to it if we have
			// TODO : Send data if we have for this tracker
		}

		ack_payload->data[0] = ESB_CONTROL_PREAMBLE;
		ack_payload->data[1] = ESB_PACKET_CONTROL_WINDOW_INFO; // Window Info (5)
		ack_payload->data[2] = tracker_window;
		memcpy(&ack_payload->data[3], &tdma_timer, sizeof(tdma_timer));
		ack_payload->length = 8;
		*has_ack_payload = true;
		if(tracker_id != WRONG_TRACKER_ID && pipe_id > 0) {
			statistics[tracker_id].packets_received++;
			if(tracker_window != current_window)
				statistics[tracker_id].windows_missed++;
			else
				statistics[tracker_id].windows_hit++;
			uint8_t packet_number = 0;
			if(data_length == 17) {
				packet_number = pdu_data[16];
			} else if(data_length == 21) {
				packet_number = pdu_data[20];
			}

			uint8_t diff = packet_number - statistics[tracker_id].last_packet_number;
			statistics[tracker_id].packets_lost += diff - 1;
			statistics[tracker_id].last_packet_number = packet_number;
		}

		// TODO : Track TDMA misses in debug mode
		/*
		if(current_slot < 24) {
			LOG_INF("Tracker broadcased in dongle's slot (%d)", current_slot);
		} else {
			if(tracker_window != current_window)
				LOG_INF("Tracker missed it's window %d != %d, slot %d", tracker_window, current_window, current_slot);
		}
		*/
	}
}

bool is_synchronized = false;

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX");
		int err = 0;
		while (!err) // zero, rx success
		{
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA) {
				return;
			} else if (err) {
				LOG_ERR("Error while reading rx packet: %d", err);
				return;
			}

			if(rx_payload.data[0] == 0xCD) {
				// Control packet received
				switch(rx_payload.data[1]) {
				case 5: // Window Info
					uint32_t current_from_slot0 = (packet_sent_time + tdma_timer_offset) & 0x7FFF;
					tracker_window = rx_payload.data[2];
					uint32_t received_from_slot0 = *((uint32_t *) &rx_payload.data[3]);
					int32_t diff = received_from_slot0 - current_from_slot0;
					int32_t new_offset = tdma_timer_offset + diff / 2;
					if(new_offset == diff)
						is_synchronized = true;
					// TODO Have a rolling average to remove outliers?
					if(skip++ == 0)
						LOG_INF("Timer offset diff received: %d, new offset %d -> %d (+%d)", diff, tdma_timer_offset, new_offset, tdma_timer_offset_static);
					tdma_timer_offset = new_offset;
					continue;
				default:
					LOG_INF("Control packet %d received", rx_payload.data[1]);
				}
			}

			if(rx_payload.pipe == 0)
				continue; // Handled in ACK handler
				
			switch (rx_payload.length)
			{
			case 21: // has sequence number & crc32
			case 17: // has sequence number
				// TODO Track packet loss
				// Fall-throught
			case 20: // has crc32
				// We ignore it
				// Fall-throught
			case 16:
				uint8_t tracker_id = rx_payload.data[1];
				if (tracker_id >= stored_trackers) // not a stored tracker
					continue;
				if (rx_payload.data[0] > ESB_CONTROL_PREAMBLE) // reserved for receiver only
					break;
				if(tdma_get_tracker_window(tracker_id) == TDMA_WRONG_WINDOW) // Tracker doesn't have a window, refuse its packets
					break;
				hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				break;
			default:
				LOG_ERR("Wrong packet length: %d", rx_payload.length);
				break;
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8}; // TODO: not used
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

void esb_deinitialize() {
	if(!esb_initialized)
		return;
	esb_initialized = false;
	esb_disable();
}

int esb_initialize(bool tx)
{
	if (esb_initialized)
		LOG_WRN("ESB already initialized");
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 30;
		config.retransmit_delay = 600;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
//		config.use_fast_ramp_up = true;
		config.ack_handler = ack_handler;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 30;
		config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
//		config.use_fast_ramp_up = true;
		config.ack_handler = ack_handler;
	}

	LOG_INF("Initializing ESB, %sX mode", tx ? "T" : "R");
	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (!err)
		esb_set_rf_channel(50);

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

inline void esb_set_addr(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = buf[5] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

void esb_add_pair(uint64_t addr, bool checksum)
{
	int id = stored_trackers;
	if (checksum)
	{
		for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
		{
			if (addr != 0 && stored_tracker_addr[i] == addr)
			{
				id = i;
			}
		}
	}
	if (id == stored_trackers)
	{
		LOG_INF("Added device on id %d with address %012llX", id, addr);
		stored_tracker_addr[id] = addr;
		sys_write(STORED_ADDR_0 + id, NULL, &stored_tracker_addr[id], sizeof(stored_tracker_addr[0]));
		stored_trackers++;
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	}
	else
	{
		LOG_INF("Device already stored with id %d", id);
	}
	if (checksum)
	{
		uint8_t buf[6] = {0};
		memcpy(buf, &addr, 6);
		uint8_t checksum = crc8_ccitt(0x07, buf, 6);
		if (checksum == 0)
			checksum = 8;
		uint64_t *receiver_addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet
		addr = (*receiver_addr & 0xFFFFFFFFFFFF) << 16;
		addr |= checksum; // Add checksum to the address
		addr |= (uint64_t)id << 8; // Add tracker id to the address
		LOG_INF("Pair the device with %016llX", addr);
	}
}

void esb_pop_pair(void)
{
	if (stored_trackers > 0)
	{
		stored_trackers--;
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		LOG_INF("Removed device on id %d with address %012llX", stored_trackers, stored_tracker_addr[stored_trackers]);
	}
	else
	{
		LOG_WRN("No devices to remove");
	}
}

void esb_clear(void)
{
	stored_trackers = 0;
	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	LOG_INF("NVS Reset");
}

// TODO: WHAT IS THIS???? WHYO MADE IT AND WHY
void esb_write_sync(uint16_t led_clock)
{
	if (!esb_initialized)
		return;
	tx_payload_sync.noack = false;
	tx_payload_sync.data[0] = (led_clock >> 8) & 255;
	tx_payload_sync.data[1] = led_clock & 255;
	esb_write_payload(&tx_payload_sync);
}

bool is_dongle_window() {
	uint32_t tdma_timer = tdma_get_timer();
	uint32_t current_slot = tdma_get_slot(tdma_timer);
	uint8_t current_window = tdma_get_window(current_slot);
	return tdma_is_dongle_window(current_slot);
}

void prepare_dongle_sate_packet() {
	tx_payload_dongle_sate.noack = false;
	tx_payload_dongle_sate.data[0] = 0xCD;
	tx_payload_dongle_sate.data[2] = 3;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&tx_payload_dongle_sate.data[3], addr, 6);
	uint8_t has_empty_windows_flag = (tdma_has_empty_windows() ? 1 : 0) << 0; // Accepts new trackers? - 0/1 bit, 1 if the dongle has empty slots to accept new tracker pairings
	uint8_t force_pairing_flag = (0) << 1; // Force pair - 0/1 bit, tells unconnected trackers to pair to this dongle even if they have pairing information saved
	tx_payload_dongle_sate.data[9] = has_empty_windows_flag | force_pairing_flag; // Dongle sate flags
	uint8_t channels_set = 0; // Channels bundle - 4 bits - which channels bundle this dongle hops on
	uint8_t channel_offset = 0 << 4; // Next channel offset - 4 bits - which channel bundle this dongle will move next after this broadcast
	tx_payload_dongle_sate.data[10] = channels_set | channel_offset; // Channels
}

static void esb_thread(void)
{
	tdma_init();

	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	for (int i = 0; i < stored_trackers; i++)
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);

	esb_set_addr();
	esb_initialize(false);
	esb_start_rx();

	bool was_dongle_window = false;

	while(1) {
		if(new_paired_address != 0) {
			esb_add_pair(new_paired_address, false);
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
			new_paired_address = 0;
		}
		if(is_dongle_window()) {
			if(!was_dongle_window) {
				was_dongle_window = true;
				for(int i = 0; i < MAX_TRACKERS; ++i) {
					if(statistics[i].packets_received > 0) {
						LOG_INF("Tracker[%d] stat. Rcvd: %d, Lost: %d, Hit: %d, Miss: %d", i, statistics[i].packets_received, statistics[i].packets_lost, statistics[i].windows_hit, statistics[i].windows_missed);
					}
					statistics[i].packets_lost = 0;
					statistics[i].packets_received = 0;
					statistics[i].windows_hit = 0;
					statistics[i].windows_missed = 0;
				}

				esb_deinitialize();
				esb_initialize(true);

				prepare_dongle_sate_packet();
				esb_write_payload(&tx_payload_dongle_sate);
				esb_start_tx();

				while(!esb_is_idle())
					k_msleep(1);
				
				esb_deinitialize();
				esb_initialize(false);
				esb_start_rx();
			}
		} else {
			was_dongle_window = false;
			k_msleep(10);
		}
	}
}
