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
#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include <esb.h>

#define ESB_CONTROL_PREAMBLE 0xCD
#define ESB_TEST_PREAMBLE 0xCF // Reserved

#define ESB_PACKET_CONTROL_PAIR_REQEST 1
#define ESB_PACKET_CONTROL_PAIR_ACCEPT 2
#define ESB_PACKET_CONTROL_DONGLE_STATUS 3
#define ESB_PACKET_CONTROL_NO_WINDOWS 4
#define ESB_PACKET_CONTROL_WINDOW_INFO 5

void event_handler(struct esb_evt const *event);
void ack_handler(uint8_t *pdu_data, uint8_t data_length, uint32_t pipe_id, struct esb_payload *ack_payload, bool *has_ack_payload);
int clocks_start(void);
int esb_initialize(bool);

void esb_set_addr(void);

int esb_get_frequency(void);

void esb_add_pair(uint64_t addr, bool checksum);
void esb_pop_pair(void);

void esb_clear(void);
void esb_write_sync(uint16_t led_clock);

#endif
