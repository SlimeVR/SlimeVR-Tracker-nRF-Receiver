
#include "globals.h"
#include "system/system.h"
#include "rssi.h"
#include "esb.h"
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

LOG_MODULE_REGISTER(rssi, LOG_LEVEL_INF);

#define WAIT_AND_RESET( m ) do { while (!m); m = 0; } while(0)
static uint32_t scan_repeat_times = 1;
static uint32_t sweeps_per_scan = 10000;

uint8_t rssi_scan_channel(uint8_t channel_number) {
	uint8_t sample;

	NRF_RADIO->FREQUENCY  = channel_number;
	NRF_RADIO->TASKS_RXEN = 1;

	WAIT_AND_RESET(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_RSSISTART = 1;
	WAIT_AND_RESET(NRF_RADIO->EVENTS_RSSIEND);

	sample = 127 & NRF_RADIO->RSSISAMPLE;

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_AND_RESET(NRF_RADIO->EVENTS_DISABLED);

	return sample;
}

void rssi_configure_radio(void) {
	NRF_RADIO->POWER  = 1;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
	NVIC_EnableIRQ(RADIO_IRQn);

	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

uint8_t rssi_scan_channel_repeat(uint8_t channel_number)
{
	uint8_t sample1;
	uint8_t max = 127;
	for (int i = 0; i <= scan_repeat_times; ++i) {
		sample1 = rssi_scan_channel(channel_number);
		// taking highest signal, but MIN because it's negative
		max = MIN(sample1, max);
	}
	return max;
}

struct ch_scan_result_t {
	uint8_t min;
	uint8_t max;
	uint32_t sum;
} ch_scan_result;

void rssi_print_sweep() {
	rssi_configure_radio();
	LOG_INF("Scanning through all channels...");
	printk("SC	CH	RSSI AVG	RSSI MIN	RSSI MAX\n");
	uint32_t scan = 0;
	while(true) {
		scan++;
		printk("Scan %d\n", scan);
		struct ch_scan_result_t scan_results[86 / 2] = {};
		for(int i = 0; i < 86 / 2; ++i) {
			scan_results[i].min = 127;
		}
		for(int sweep = 0; sweep < sweeps_per_scan; ++sweep) {
			for(int ch = 0; ch <= 84; ch += 2) {
				uint8_t rssi = rssi_scan_channel_repeat(ch);
				scan_results[ch / 2].min = MIN(scan_results[ch / 2].min, rssi);
				scan_results[ch / 2].max = MAX(scan_results[ch / 2].max, rssi);
				scan_results[ch / 2].sum += rssi;
			}
			k_msleep(1);
		}
		for(int i = 0; i < 86 / 2; ++i) {
			printk("%d	%d	%d	%d	%d\n", scan, i * 2, scan_results[i].sum / sweeps_per_scan, scan_results[i].min, scan_results[i].max);
			k_msleep(5);
		}
		k_msleep(100);
	}
}