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

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/crc.h>

#include "esb.h"

static struct esb_payload rx_payload;
//static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);
//static struct esb_payload tx_payload_timer = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0);

uint8_t pairing_buf[8] = {0};
static uint8_t discovered_trackers[256] = {0};

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_packet_filter_thread(void);
K_THREAD_DEFINE(esb_packet_filter_thread_id, 256, esb_packet_filter_thread, NULL, NULL, NULL, 6, 0, 0);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, 6, 0, 0);

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
	// make tx payload for ack here
		if (!esb_read_rx_payload(&rx_payload)) // zero, rx success
		{
			switch (rx_payload.length)
			{
			case 8:
				LOG_INF("RX Pairing Packet");
				memcpy(pairing_buf, rx_payload.data, 8);
				esb_write_payload(&tx_payload_pair); // Add to TX buffer
				break;
			case 16:
				uint8_t imu_id = rx_payload.data[1];
				if (imu_id >= stored_trackers) // not a stored tracker
					return;
				if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
				{
					discovered_trackers[imu_id]++;
					return;
				}
				if (rx_payload.data[0] > 223) // reserved for receiver only
					break;
				hid_write_packet_n(rx_payload.data, rx_payload.rssi); // write to hid endpoint
				break;
			default:
				break;
			}
		}
		else
		{
			LOG_ERR("Error while reading rx packet");
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
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 8;
		// config.retransmit_delay = 600;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 8;
		// config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

static void esb_deinitialize(void)
{
	if (esb_initialized)
		esb_disable();
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
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
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static bool esb_pairing = false;
static bool esb_paired = false;

void esb_pair(void)
{
	LOG_INF("Pairing");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	esb_pairing = true;
	while (esb_pairing)
	{
		uint64_t found_addr = (*(uint64_t *)pairing_buf >> 16) & 0xFFFFFFFFFFFF;
		uint16_t send_tracker_id = stored_trackers; // Use new tracker id
		for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
		{
			if (found_addr != 0 && stored_tracker_addr[i] == found_addr)
			{
				//LOG_INF("Found device linked to id %d with address %012llX", i, found_addr);
				send_tracker_id = i;
			}
		}
		uint8_t checksum = crc8_ccitt(0x07, &pairing_buf[2], 6); // make sure the packet is valid
		if (checksum == 0)
			checksum = 8;
		if (checksum == pairing_buf[0] && found_addr != 0 && send_tracker_id == stored_trackers && stored_trackers < MAX_TRACKERS) // New device, add to NVS
		{
			LOG_INF("Added device on id %d with address %012llX", stored_trackers, found_addr);
			stored_tracker_addr[stored_trackers] = found_addr;
			sys_write(STORED_ADDR_0+stored_trackers, NULL, &stored_tracker_addr[stored_trackers], sizeof(stored_tracker_addr[0]));
			stored_trackers++;
			sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		}
		if (checksum == pairing_buf[0] && send_tracker_id < MAX_TRACKERS) // Make sure the dongle is not full
			tx_payload_pair.data[0] = pairing_buf[0]; // Use checksum sent from device to make sure packet is for that device
		else
			tx_payload_pair.data[0] = 0; // Invalidate packet
		tx_payload_pair.data[1] = send_tracker_id; // Add tracker id to packet
		//esb_flush_rx();
		//esb_flush_tx();
		//esb_write_payload(&tx_payload_pair); // Add to TX buffer
		k_msleep(10);
	}
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
	esb_disable();
	esb_receive();
}

void esb_reset_pair(void)
{
	esb_deinitialize(); // make sure esb is off
	esb_paired = false;
}

void esb_finish_pair(void)
{
	esb_pairing = false;
}

void esb_clear(void)
{
	stored_trackers = 0;
	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	LOG_INF("NVS Reset");
	esb_reset_pair();
}

// TODO:
void esb_write_sync(uint16_t led_clock)
{
	if (!esb_initialized || !esb_paired)
		return;
	tx_payload_sync.noack = false;
	tx_payload_sync.data[0] = (led_clock >> 8) & 255;
	tx_payload_sync.data[1] = led_clock & 255;
	esb_write_payload(&tx_payload_sync);
}

// TODO:
void esb_receive(void)
{
	esb_set_addr_paired();
	esb_paired = true;
}

static void esb_packet_filter_thread(void)
{
	memset(discovered_trackers, 0, sizeof(discovered_trackers));
	while (1) // reset count if its not above threshold
	{
		k_msleep(1000);
		for (int i = 0; i < 256; i++)
			if (discovered_trackers[i] < DETECTION_THRESHOLD)
				discovered_trackers[i] = 0;
	}
}

static void esb_thread(void)
{
	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	if (stored_trackers)
		esb_paired = true;
	for (int i = 0; i < stored_trackers; i++)
		sys_read(STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);

	if (esb_paired)
	{
		esb_receive();
		esb_initialize(false);
		esb_start_rx();
	}

	while (1)
	{
		if (!esb_paired)
		{
			esb_pair();
			esb_initialize(false);
			esb_start_rx();
		}
		k_msleep(100);
	}
}
