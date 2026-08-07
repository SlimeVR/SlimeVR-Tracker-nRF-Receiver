
#pragma once

#include <stdint.h>
#include <esb.h>

uint8_t rssi_scan_channel_repeat(uint8_t channel);
uint8_t rssi_scan_channel(uint8_t channel_number);

void rssi_print_sweep();