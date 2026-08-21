
#pragma once

#include <stdint.h>
#include <esb.h>

uint8_t rssi_scan_channel(uint8_t channel_number);
uint8_t rssi_scan_channel_repeat(uint8_t channel);
uint8_t ed_scan_channel_repeat(uint8_t channel);

void scan_print_sweep();