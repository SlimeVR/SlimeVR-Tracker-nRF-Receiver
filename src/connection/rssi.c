
#include "globals.h"
#include "system/system.h"
#include "rssi.h"
#include "esb.h"
#include "nettests.h"
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

LOG_MODULE_REGISTER(rssi, LOG_LEVEL_INF);

#define WAIT_AND_RESET( m ) do { while (!m); m = 0; } while(0)
static uint32_t scan_repeat_times = 1;
static uint32_t sweeps_per_scan = 1000;

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

void sweep_configure_radio(void) {
#if ED_SCAN
	NRF_RADIO->MODE = RADIO_MODE_MODE_Ieee802154_250Kbit;
#endif
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

uint8_t ed_scan_channel_repeat(uint8_t channel_number)
{
	const uint8_t ED_RSSISCALE = 4;
	uint8_t sample;

	NRF_RADIO->FREQUENCY  = channel_number;
	NRF_RADIO->TASKS_RXEN = 1;
	NRF_RADIO->EDCNT = scan_repeat_times - 1;

	WAIT_AND_RESET(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_EDSTART = 1;
	WAIT_AND_RESET(NRF_RADIO->EVENTS_EDEND);

	sample = MIN(NRF_RADIO->EDSAMPLE * ED_RSSISCALE, 255);

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_AND_RESET(NRF_RADIO->EVENTS_DISABLED);

	return sample;
}

struct ch_scan_result_t {
	uint8_t min;
	uint8_t max;
	uint32_t sum;
} ch_scan_result;

#define PRINT_PROGRESS false
void scan_print_sweep() {
	sweep_configure_radio();
#if RSSI_SCAN && ED_SCAN
	LOG_ERR("RSSI scan can be performed with ED scan");
	return;
#endif
	bool rssiTest = false;
#if RSSI_SCAN
	rssiTest = true;
#endif
	LOG_INF("Scanning %s through all channels... %d by %d samples", rssiTest ? "RSSI" : "ED", scan_repeat_times, sweeps_per_scan);
	printk("SC\tCH\tAVG\tMIN\tMAX\n");
	uint32_t scan = 0;
	while(true) {
		scan++;
		printk("Scan %d\n", scan);
		struct ch_scan_result_t scan_results[86 / 2] = {};
		for(int i = 0; i < 86 / 2; ++i) {
			scan_results[i].min = 127;
		}
#if PRINT_PROGRESS
		int nextPercent = 10;
#endif
		for(int sweep = 0; sweep < sweeps_per_scan; ++sweep) {
			for(int ch = 0; ch <= 84; ch += 2) {
#if RSSI_SCAN
				uint8_t val = rssi_scan_channel_repeat(ch);
#elif ED_SCAN
				uint8_t val = ed_scan_channel_repeat(ch);
#endif
				scan_results[ch / 2].min = MIN(scan_results[ch / 2].min, val);
				scan_results[ch / 2].max = MAX(scan_results[ch / 2].max, val);
				scan_results[ch / 2].sum += val;
			}
			k_msleep(1);
#if PRINT_PROGRESS
			uint8_t percent = ((sweep + 1) * 100) / sweeps_per_scan;
			if(percent >= nextPercent) {
				printk("%d%%%c", percent, nextPercent == 100 ? '\n' : '.');
				nextPercent += 10;
			}
#endif
		}
		for(int i = 0; i < 86 / 2; ++i) {
			uint8_t avg = (uint8_t) (scan_results[i].sum / sweeps_per_scan);
			printk("%d\t%d\t%d\t%d\t%d\n", scan, i * 2, avg, scan_results[i].min, scan_results[i].max);
			k_msleep(5);
		}
		k_msleep(100);
	}
}