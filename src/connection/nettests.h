
#pragma once

#include <stdint.h>
#include "esb.h"

#define SWEEP_TEST false
#define RSSI_SCAN false

void sweep_short_packet();
// I fucking hate pointers - Eiren
void sweep_control_test_rcvd(struct esb_payload rx_payload);
void sweep_run();