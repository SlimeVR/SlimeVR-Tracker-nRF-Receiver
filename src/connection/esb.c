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
#include "rssi.h"

#include "esb.h"
#include "nettests.h"

#if RSSI_SCAN
#include "rssi.h"
#endif

#define FREQUENCY_HOPPING false

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, ESB_THREAD_PRIORITY, 0, 0);

static struct esb_payload rx_payload;
static struct esb_payload tx_payload_dongle_sate = ESB_EMPTY_PAYLOAD(0, 16);

static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static const uint8_t ESB_ALLOWED_CHANNEL_BUNDLES[] = {ESB_CHANNELS};
static uint8_t occupied_channels[sizeof(ESB_ALLOWED_CHANNEL_BUNDLES)] = {0};

static bool esb_initialized = false;
static bool accepts_pairing = true;
static uint8_t currentChannelBundle = 0;
static enum dongle_state_t dongle_state = CHANNEL_SELECT;
static uint64_t channel_discovery_time = 0;
static uint8_t currentESBChannel = ESB_RIMARY_ADVERTISEMENT_CHANNEL;
static bool write_stored_trackers = false;
uint8_t stored_trackers = 0;
uint64_t stored_tracker_addr[MAX_TRACKERS] = {0};

struct con_stat {
	uint8_t packets_received;
	uint8_t packets_lost;
	uint8_t windows_hit;
	uint8_t windows_missed;
	uint8_t last_packet_number;
	uint8_t repeat_packets;
	uint8_t max_gap;
	uint8_t last_rotation_packet;
	uint8_t max_rotation_gap;
};

// struct packet_stat {
// 	uint8_t tracker_id;
// 	uint32_t timer;
// 	uint8_t rcv_window;
// 	uint8_t corect_window;
// };

bool is_rotation_packet(uint8_t pid) {
	return pid == 1 || pid == 2 || pid == 4 || pid == 6 || pid == 7;
}

struct con_stat statistics[MAX_TRACKERS];
// struct packet_stat packets_statistics[2048];
// uint16_t next_packet_statistics = 0;

// Use this to generate ACK packet to send to tracker when we receive data from it
// Ideally, it should return as fast as possible
// WARNING Also must not execute any sys calls like sys_write and etc. or it will crash!
// TODO Split into multiple functions
void ack_handler(uint8_t *pdu_data, uint8_t data_length, uint32_t pipe_id, struct esb_payload *ack_payload, bool *has_ack_payload) {
	#if SWEEP_TEST
		return;
	#endif
	if(pipe_id != 0) {
		const uint8_t packet_number = pdu_data[0]; // Sequence number
		const uint8_t packet_id = pdu_data[1];
		const uint8_t tracker_id = pdu_data[2];
		ack_payload->data[0] = packet_number;
		switch(packet_id) {
		case ESB_PACKET_CONTROL_PAIR_REQEST:
			if(data_length < 15) {
				LOG_WRN("Short pairing packet received: %d byes", data_length);
				return;
			}
			ack_payload->data[1] = ESB_PACKET_CONTROL_PAIR_RESPONSE;
			uint64_t tracker_hwid = *((uint64_t *) &pdu_data[3]) & 0xFFFFFFFFFFFF;
			uint8_t response = accepts_pairing ? esb_add_pair(tracker_hwid) : ESB_STATUS_NOT_ACCEPTING; // TODO Can we???
			if(response >= ESB_STATUS_ERROR) {
				LOG_INF("Can't add tracker %012llX, reason: %d", tracker_hwid, response);
			} else {
				LOG_INF("Added tracker %012llX with tracker id %d", tracker_hwid, response);
			}
			ack_payload->data[2] = response;
			uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
			memcpy(&ack_payload->data[3], addr, 6);
			memcpy(&ack_payload->data[9], &tracker_hwid, 6);
			ack_payload->length = 15;
			*has_ack_payload = true;
			return;
		case ESB_PACKET_DONGLE_CONNECT:
			if(data_length < 11) {
				LOG_WRN("Short connect packet received: %d byes", data_length);
				return;
			}
			ack_payload->data[1] = ESB_PACKET_DONGLE_CONNECT_REPLY;
			tracker_hwid = *((uint64_t *) &pdu_data[3]) & 0xFFFFFFFFFFFF;
			memcpy(&ack_payload->data[3], &tracker_hwid, 6);
			ack_payload->data[9] = ESB_VERSION;
			ack_payload->data[10] = PROTOCOL_VERSION;
			uint8_t real_tracker_id = esb_get_tracker_id(tracker_hwid);
			if(real_tracker_id == WRONG_TRACKER_ID) {
				ack_payload->data[2] = ESB_STATUS_NOT_PAIRED;
			} else {
				uint8_t tracker_window = tdma_get_or_allocate_tracker_window(tracker_id);
				if(tracker_window == TDMA_WRONG_WINDOW) {
					ack_payload->data[2] = ESB_STATUS_NO_SLOTS;
				} else {
					ack_payload->data[2] = tracker_id;
				}
			}
			ack_payload->length = 11;
			LOG_INF("Connect packet received from %012llX, response: %d", tracker_hwid, ack_payload->data[2]);
			*has_ack_payload = true;
			return;
		}
		// Check tracker's timing window and send its offset for TDMA
		uint32_t tdma_timer = tdma_get_timer();
		uint8_t tracker_window = 0;
		if(tracker_id == WRONG_TRACKER_ID) {
			// Not tracker packet, skip
			return;
		}
		tracker_window = tdma_get_tracker_window(tracker_id);
		if(tracker_window == TDMA_WRONG_WINDOW) {
			// Tracker isn't assigned a window, ask it to send connect packet
			ack_payload->data[1] = ESB_PACKET_DONGLE_RECONNECT;
			ack_payload->data[2] = ESB_VERSION;
			ack_payload->data[3] = PROTOCOL_VERSION;
			ack_payload->length = 4;
			*has_ack_payload = true;
			return;
		}
		uint32_t current_slot = tdma_get_slot(tdma_timer);
		uint8_t current_window = tdma_get_window(current_slot);
		if(!tdma_is_dongle_window(current_slot) && current_window == tracker_window) {
			// Tracker sent data at the correct time, we can send data to it if we have
			// TODO : Send data if we have for this tracker
		} // else {
			// Send Window Info (5) if we don't have anything better
			// TODO : We should send one at least once every 20-100 packets to prevent timer drift
			ack_payload->data[1] = ESB_PACKET_CONTROL_WINDOW_INFO; // Window Info (235)
			ack_payload->data[2] = tracker_window;
			memcpy(&ack_payload->data[3], &tdma_timer, sizeof(tdma_timer));
			ack_payload->data[7] = 0; // For future: next channel
			ack_payload->length = 8;
			*has_ack_payload = true;
		// }

		struct con_stat* stat = &statistics[tracker_id];

		if(packet_number != stat->last_packet_number) {
			stat->packets_received++;
			if(tracker_window != current_window)
				stat->windows_missed++;
			else
				stat->windows_hit++;

			uint8_t diff = packet_number - stat->last_packet_number;
			stat->packets_lost += diff - 1;
			stat->last_packet_number = packet_number;
			stat->max_gap = MAX(stat->max_gap, diff - 1);
			if(is_rotation_packet(packet_id)) {
				uint8_t r_diff = packet_number - stat->last_rotation_packet;
				stat->max_rotation_gap = MAX(stat->max_rotation_gap, r_diff - 1);
				stat->last_rotation_packet = packet_number;
			}
			if(diff > 3) {
				LOG_WRN("Tracker %d lost %d packets in a row (max gap %d, max r-gap %d)", tracker_id, diff-1, stat->max_gap, stat->max_rotation_gap);
			}
		} else {
			stat->repeat_packets++;
		}

		// uint16_t packet_n = next_packet_statistics++;
		// packets_statistics[packet_n].tracker_id = tracker_id;
		// packets_statistics[packet_n].corect_window = tracker_window;
		// packets_statistics[packet_n].rcv_window = current_window;
		// packets_statistics[packet_n].timer = tdma_timer;
		// LOG_INF("P %d T %d @ %d t (%d / %d w, %d s) (%d off) N %d", pdu_data[0], tracker_id, tdma_timer, current_window, tracker_window, current_slot, tdma_timer - tdma_get_slot_time(current_slot), packet_number);
		if(current_slot < TDMA_DONGLE_SLOTS) {
			LOG_WRN("Tracker %d broadcased in dongle's slot (%d)", tracker_id, current_slot);
		} else {
			if(tracker_window != current_window)
				LOG_WRN("Tracker %d missed it's window (expected %d, got %d), slot %d", tracker_id, tracker_window, current_window, current_slot);
		}
	}
}

// TODO Split into multiple functions
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
				LOG_WRN("Error while reading rx packet: %d", err);
				return;
			}
			if(rx_payload.length < 3) {
#if SWEEP_TEST
				sweep_short_packet();
#else
				LOG_WRN("Too short packet received");
#endif
				return;
			}

			if(rx_payload.pipe == 0 && rx_payload.data[0] == 0xCD && rx_payload.data[1] == 3) {
				uint64_t dongle_hwid = *((uint64_t *) &rx_payload.data[2]) & 0xFFFFFFFFFFFF;
				LOG_WRN("Outdated Dongle Status packet received during normal operation from %012llX on channel %d. Expect interference.", dongle_hwid, currentESBChannel);
				return;
			}
			
			if(rx_payload.data[1] > ESB_PACKET_DONGLE_PACKETS) {
				// Packet for dongle received
				switch(rx_payload.data[1]) {
				case ESB_PACKET_CONTROL_DONGLE_STATUS:
					#if SWEEP_TEST
						return;
					#endif
					if(rx_payload.length < 11) {
						LOG_ERR("Too short packet received");
						return;
					}
					uint64_t dongle_hwid = *((uint64_t *) &rx_payload.data[2]) & 0xFFFFFFFFFFFF;
					uint8_t channel = rx_payload.data[8];
					for(int i = 0; i < sizeof(occupied_channels); ++i) {
						if(occupied_channels[i] == 0) {
							occupied_channels[i] = channel;
							LOG_INF("Found neighboring dongle %012llX on channel %d with RSSI -%d", dongle_hwid, channel, rx_payload.rssi);
							break;
						} else if(occupied_channels[i] == channel) {
							break;
						}
					}
					if(occupied_channels[sizeof(occupied_channels) - 1] != 0) {
						LOG_ERR("No empty channels left on the air, terminating search.");
						// TODO Restart search later?
						dongle_state = NO_CHANNELS;
						return;
					}
				break;
				case ESB_PACKET_CONTROL_TEST:
					sweep_control_test_rcvd(rx_payload);
				break;
				case ESB_PACKET_DONGLE_CONNECT:
				case ESB_PACKET_CONTROL_PAIR_REQEST:
					// Handled in ack handler
				break;
				default:
					#if !SWEEP_TEST
						LOG_INF("Control packet %d received", rx_payload.data[1]);
					#endif
				}
				break;
			}

			if(rx_payload.pipe != 0) {
				if (rx_payload.length >= 17) {
					uint8_t tracker_id = rx_payload.data[2];
					if (tracker_id >= stored_trackers) // not a stored tracker
						continue;
					if(tdma_get_tracker_window(tracker_id) == TDMA_WRONG_WINDOW) // Tracker doesn't have a window, refuse its packets
						break;
					if(rx_payload.data[1] == 3) { // status
						// Fill in packet lost statistics in status packet
						rx_payload.data[5] = statistics[tracker_id].packets_received;
						rx_payload.data[6] = statistics[tracker_id].packets_lost;
						rx_payload.data[7] = statistics[tracker_id].windows_hit;
						rx_payload.data[8] = statistics[tracker_id].windows_missed;
						// Received from the tracker
						rx_payload.data[13] = statistics[tracker_id].repeat_packets;
						statistics[tracker_id].packets_lost = 0;
						statistics[tracker_id].packets_received = 0;
						statistics[tracker_id].windows_hit = 0;
						statistics[tracker_id].windows_missed = 0;
						statistics[tracker_id].repeat_packets = 0;
						statistics[tracker_id].max_gap = 0;
						statistics[tracker_id].max_rotation_gap = 0;
					}
					hid_write_packet_n(rx_payload.data + 1, rx_payload.rssi, 16); // write to hid endpoint
					break;
				} else {
					LOG_ERR("Wrong packet length: %d", rx_payload.length);
					break;
				}
			}
		}
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

void esb_deinitialize() {
	if(!esb_initialized)
		return;
	esb_initialized = false;
	esb_disable();
}

int esb_initialize(bool tx, bool advertize)
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
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = 435;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = false;
		config.ack_handler = ack_handler;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		config.crc = SWEEP_TEST ? ESB_CRC_OFF : ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = 435;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = false;
		config.ack_handler = ack_handler;
	}

	err = esb_init(&config);

	if (!err)
	{
		esb_set_base_address_0(base_addr_0);
		esb_set_base_address_1(base_addr_1);
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
		esb_set_rf_channel(advertize ? ESB_RIMARY_ADVERTISEMENT_CHANNEL : currentESBChannel);
	}
	else
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}
	int32_t ch;
	esb_get_rf_channel(&ch);
	LOG_INF("Initialized ESB, %sX mode ch %d, address %08X", tx ? "T" : "R", ch, *((uint32_t *) &base_addr_1[0]));

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

int esb_get_frequency(void) {
	uint32_t channel;
	esb_get_rf_channel(&channel);
	return 2400UL + channel; // MHz
}

uint8_t esb_add_pair(uint64_t addr)
{
	if(addr == 0)
		return ESB_STATUS_ERROR;
	int id = stored_trackers;
	for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
	{
		if (stored_tracker_addr[i] == addr)
		{
			id = i;
		}
	}
	if (id == stored_trackers)
	{
		if(id == (sizeof(stored_tracker_addr) / sizeof(stored_tracker_addr[0])))
			return ESB_STATUS_NO_SLOTS;
		LOG_INF("Added device on id %d with address %012llX", id, addr);
		stored_tracker_addr[id] = addr;
		stored_trackers++;
		write_stored_trackers = true;
	}
	else
	{
		LOG_INF("Device %012llX is already stored with id %d", addr, id);
	}
	return id;
}

uint8_t esb_get_tracker_id(uint64_t addr) {
	for (int i = 0; i < stored_trackers; i++) {
		if (stored_tracker_addr[i] == addr) {
			return i;
		}
	}
	return WRONG_TRACKER_ID;
}

void esb_clear(void)
{
	stored_trackers = 0;
	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	LOG_INF("NVS Reset");
}

bool is_dongle_window() {
	uint32_t tdma_timer = tdma_get_timer();
	uint32_t current_slot = tdma_get_slot(tdma_timer);
	return tdma_is_dongle_window(current_slot);
}

void prepare_dongle_sate_packet() {
	tx_payload_dongle_sate.noack = true;
	tx_payload_dongle_sate.data[0] = 0; // Sequence is always 0
	tx_payload_dongle_sate.data[1] = ESB_PACKET_CONTROL_DONGLE_STATUS;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&tx_payload_dongle_sate.data[2], addr, 6);
	tx_payload_dongle_sate.data[8] = ESB_ALLOWED_CHANNEL_BUNDLES[currentChannelBundle];

	uint8_t has_empty_windows_flag = (tdma_has_empty_windows() ? 1 : 0) << 0; // Accepts new trackers? - 0/1 bit, 1 if the dongle has empty slots to accept new tracker pairings
	uint8_t force_pairing_flag = (0) << 1; // Force pair - 0/1 bit, tells unconnected trackers to pair to this dongle even if they have pairing information saved
	tx_payload_dongle_sate.data[9] = has_empty_windows_flag | force_pairing_flag; // Dongle sate flags
	
	uint32_t tdma_timer = tdma_get_timer();
	memcpy(&tx_payload_dongle_sate.data[10], &tdma_timer, sizeof(tdma_timer));
	tx_payload_dongle_sate.data[14] = ESB_VERSION;
	tx_payload_dongle_sate.data[15] = PROTOCOL_VERSION;
}

void pick_channels() {
	channel_discovery_time = k_uptime_get() + ESB_CHANNEL_DISCOVERY_TIME;
	while(dongle_state == CHANNEL_SELECT && channel_discovery_time > k_uptime_get()) {
		// Listem to advertisement packets and wait for our spot
		k_msleep(10);
	}
	if(dongle_state == CHANNEL_SELECT) {
		// Channel picked successfully, we rockin'
		for(int i = 0; i < sizeof(ESB_ALLOWED_CHANNEL_BUNDLES); ++i) {
			bool occupied = false;
			for(int j = 0; j < sizeof(occupied_channels); ++j) {
				if(occupied_channels[j] == 0) {
					break;
				}
				if(ESB_ALLOWED_CHANNEL_BUNDLES[i] == occupied_channels[j]) {
					occupied = true;
					break;
				}
			}
			if(!occupied) {
				currentChannelBundle = i;
				currentESBChannel = ESB_ALLOWED_CHANNEL_BUNDLES[i];
				LOG_INF("Found an empty channel %d", currentESBChannel);
				esb_deinitialize();
				esb_initialize(false, false);
				dongle_state = ACTIVE;
				return;
			}
		}
	}
}

static void esb_thread(void)
{
#if SWEEP_TEST || RSSI_SCAN
	k_msleep(5000);
#endif
	tdma_init();

	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	for (int i = 0; i < stored_trackers; i++)
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);

#if RSSI_SCAN
	rssi_print_sweep();
#endif

	esb_set_addr();
	esb_initialize(false, true);
#if SWEEP_TEST
	sweep_run();
#endif
	esb_start_rx();
	pick_channels();

	bool was_dongle_window = false;
	uint8_t status_timing_shift = 0;
	uint32_t last_slot = 0;
	uint32_t current_slot = 0;

	while(dongle_state == ACTIVE) {
		if(write_stored_trackers) {
			write_stored_trackers = false;
			for (int i = 0; i < stored_trackers; i++)
				sys_write(STORED_ADDR_0 + i, NULL, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
			sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		}
		// if(new_paired_address != 0) {
		// 	esb_add_pair(new_paired_address, false);
		// 	set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		// 	new_paired_address = 0;
		// }
#if FREQUENCY_HOPPING
		while(last_slot == current_slot) {
			k_sleep(K_TICKS(1));
			current_slot = tdma_get_slot(tdma_get_timer());
		}
#endif
		last_slot = current_slot;
		if(is_dongle_window()) {
			if(!was_dongle_window) {
				was_dongle_window = true;

				esb_deinitialize();
				esb_initialize(true, true);

				// TODO : Send on secondary channel
				for(int i = 0; i < 3; ++i) {
					prepare_dongle_sate_packet();
					esb_write_payload(&tx_payload_dongle_sate);
					esb_start_tx();
					while(!esb_is_idle())
						k_sleep(K_TICKS(1));
					k_sleep(K_TICKS(1 + (status_timing_shift++ % 3)));
				}
				
				esb_deinitialize();
				esb_initialize(false, false);
				esb_start_rx();
				if(!is_dongle_window()) {
					LOG_WRN("Took too long in dongle window!");
				}
			}
		} else {
			was_dongle_window = false;
#if FREQUENCY_HOPPING
			esb_stop_rx();
			currentESBChannel = ESB_ALLOWED_CHANNEL_BUNDLES[(current_slot) % 10];
			esb_set_rf_channel(currentESBChannel);
			esb_start_rx();
#else
			k_msleep(1);
#endif
		}
	}
}
