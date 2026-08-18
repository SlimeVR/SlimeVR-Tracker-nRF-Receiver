
#include "globals.h"
#include "nettests.h"

#if SWEEP_TEST
#include <zephyr/sys/crc.h>

LOG_MODULE_REGISTER(net_test, LOG_LEVEL_INF);

static const uint32_t sweep_count = 10000;
static const uint32_t sweep_margin = 9000;
static uint8_t sweeping_bundle = 0;
static uint8_t sweeping_rx_errors = 0;
static uint32_t sweeping_packets_received = 0;
static uint8_t sweeping_lowest_rssi = 127;
static uint8_t sweeping_highest_rssi = 0;
static uint32_t sweeping_rssi_sum = 0;
static uint32_t sweeping_rsv_min = sweep_count;
static uint32_t sweeping_rsv_max = 0;
static bool sweeping_advance_bundle = false;
static uint64_t sweep_last_hwid = 0;
static uint32_t sweep_max_gap = 0;
static uint32_t sweep_crc_errors = 0;
static uint32_t sweep_last_j = 0;
static uint8_t sweep_lowest_bat = 127;

void sweep_short_packet() {
    sweeping_rx_errors++;
}

void sweep_control_test_rcvd(struct esb_payload rx_payload) {
    #if SWEEP_TEST
        if(rx_payload.length < 16) {
            sweep_short_packet();
            return;
        }
        uint8_t ch = rx_payload.data[2];
        if(ch == sweeping_bundle) {
            uint16_t crc_rsvd = *((uint16_t *) &rx_payload.data[14]) & 0xFFFF;
            uint16_t crc = crc16_ansi(&rx_payload.data[0], 14);
            if(crc != crc_rsvd) {
                sweep_crc_errors++;
                return; // Can't continue, the data is invalid anyways
            }

            uint32_t j = *((uint32_t *) &rx_payload.data[3]) & 0xFFFFFFFF;
            uint64_t hwid = *((uint64_t *) &rx_payload.data[7]) & 0xFFFFFFFFFFFF;
            uint8_t bat = rx_payload.data[13];
            if((bat & 0x80) > 0) {
                sweep_lowest_bat = MIN(sweep_lowest_bat, bat & 0x7F);
            }
            if(sweep_last_hwid != hwid) {
                LOG_INF("Found tracker HWID %012llX, battery %d", hwid, bat);
                sweep_last_hwid = hwid;
            }
            if(j > sweep_last_j) {
                uint32_t gap = j - sweep_last_j - 1;
                sweep_max_gap = MAX(sweep_max_gap, gap);
            }
            sweep_last_j = j;
            sweeping_packets_received++;
            sweeping_highest_rssi = MAX(sweeping_highest_rssi, rx_payload.rssi);
            sweeping_lowest_rssi = MIN(sweeping_lowest_rssi, rx_payload.rssi);
            sweeping_rsv_max = MAX(sweeping_rsv_max, j);
            sweeping_rsv_min = MIN(sweeping_rsv_min, j);
            sweeping_rssi_sum += rx_payload.rssi;
            if(j > sweep_margin) {
                sweeping_advance_bundle = true;
            }
        }
    #endif
}

void sweep_run() {
    LOG_INF("Doing sweeping test...");
	printk("CH	RCVD	SENT	RSSI AVG	RSSI MIN	RSSI MAX	RX ERR	MAX GAP	CRC ERR\n");
	while(true) {
		for(sweeping_bundle = 0; sweeping_bundle <= 84; sweeping_bundle += 2) {
			uint8_t channel = sweeping_bundle;
			while(!esb_is_idle())
				k_sleep(K_TICKS(1));
			esb_set_rf_channel(channel);
			esb_start_rx();
			while(!sweeping_advance_bundle) {
				k_msleep(5);
            }
			esb_stop_rx();
			uint32_t sent = sweeping_rsv_max - sweeping_rsv_min + 1;
			printk("%2d	%3d	%3d	%2d	%2d	%2d	%d	%d	%d\n",
				channel, sweeping_packets_received, sent, sweeping_packets_received > 0 ? sweeping_rssi_sum / sweeping_packets_received : 0,
				sweeping_lowest_rssi, sweeping_highest_rssi, sweeping_rx_errors, sweep_max_gap, sweep_crc_errors);
			sweeping_advance_bundle = false;
			sweeping_highest_rssi = 0;
			sweeping_lowest_rssi = 127;
			sweeping_packets_received = 0;
			sweeping_rssi_sum = 0;
			sweeping_rsv_max = 0;
			sweeping_rsv_min = sweep_count;
			sweeping_rx_errors = 0;
            sweep_max_gap = 0;
            sweep_crc_errors = 0;
            sweep_lowest_bat = 127;
		}
	}
}
#else
void sweep_short_packet() {};
void sweep_control_test_rcvd(struct esb_payload rx_payload) {};
void sweep_run() {};
#endif