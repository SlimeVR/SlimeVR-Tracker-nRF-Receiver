

#include "globals.h"
#include "system/system.h"
#include "rssi.h"
#include "esb.h"
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

LOG_MODULE_REGISTER(rssi, LOG_LEVEL_INF);

#define RSSI_NO_SIGNAL              127    /**< Minimum value of RSSISAMPLE */
#define WAIT_FOR( m ) do { while (!m); m = 0; } while(0)
static uint32_t scan_repeat_times = 1000;

uint8_t rssi_scan_channel(uint8_t channel_number)
{
	uint8_t sample;

	NRF_RADIO->FREQUENCY  = channel_number;
	NRF_RADIO->TASKS_RXEN = 1;

	WAIT_FOR(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_RSSISTART = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_RSSIEND);

	sample = 127 & NRF_RADIO->RSSISAMPLE;

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_DISABLED);

	return sample;
}

uint8_t rssi_scan_channel_repeat(uint8_t channel_number)
{
	uint8_t sample1;
	uint8_t sample2;
	uint8_t sample3;
	uint8_t max = RSSI_NO_SIGNAL;
	for (int i = 0; i <= scan_repeat_times; ++i) {
		sample1 = rssi_scan_channel(channel_number);
		sample2 = rssi_scan_channel(channel_number+1);
		sample3 = rssi_scan_channel(channel_number-1);
		// taking minimum since sample = -dBm.
		max = MIN(sample3, MIN(sample2, MIN(sample1, max)));
		k_msleep(1);
	}
	return max;
}

void rssi_print_sweep() {
	uint8_t channels[] = {ESB_RIMARY_ADVERTISEMENT_CHANNEL, ESB_SECONDARY_ADVERTISEMENT_CHANNEL, ESB_CHANNELS};
	LOG_INF("Scanning through %d channels...", sizeof(channels));
	for(int i = 0; i < sizeof(channels); ++i) {
		uint8_t rssi = rssi_scan_channel_repeat(channels[i]);
		LOG_INF("Channel: %2d, RSSI: %3d", channels[i], rssi);
		k_msleep(300);
	}
}