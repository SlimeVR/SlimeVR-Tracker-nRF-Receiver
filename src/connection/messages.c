/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

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
#include "messages.h"
#include "esb.h"
#include "hid.h"

LOG_MODULE_REGISTER(dongle_messages, LOG_LEVEL_INF);

static bool sent_data = false;

void send_dongle_info() {
    packet_dongle_into_t dongle_info;
    dongle_info.packet_id = SERVER_PACKET_DONGLE_INFO;
    // Basic info
    dongle_info.data_type = DATA_BASIC_INFO;
    dongle_info.data_length = sizeof(packet_dongle_info_basic_t);
    packet_dongle_info_basic_t basic_info;
    basic_info.hwid = *((uint64_t *)NRF_FICR->DEVICEADDR);
    basic_info.dongle_type = 1;
    basic_info.dongle_class = 1;
    basic_info.dongle_hardware_type = HARDWARE_BUTTERFLY_DONGLE;
    basic_info.dongle_hardware_revision = 1;
    basic_info.protocol_version = P_VERSION_LEGACY;
    memcpy(dongle_info.data_array, &basic_info, sizeof(packet_dongle_info_basic_t));
    hid_write_packet_n((uint8_t *) &dongle_info, sizeof(packet_dongle_into_t));
    // TODO Other info
}

void send_trackers_list() {
    // TODO Track and send trackers list
}

void process_server_hello(uint8_t * data, int length) {
    if(length < sizeof(packet_server_hello_t)) {
        LOG_WRN("Too short SERVER HELLO (%d) packet: %d bytes", SERVER_PACKET_SERVER_HELLO, length);
        return;
    }
    packet_server_hello_t server_hello = *(packet_server_hello_t * ) &data[0];
    LOG_INF("Server hello: protocol %d, server type: %d, flags: %d, time %d", server_hello.protocol_version, server_hello.server_type, data[4], server_hello.server_time);
    if(server_hello.flag_send_all || !sent_data) {
        sent_data = true;
        //send_dongle_info();
        //send_trackers_list();
    }
}

void hid_dongle_message(uint8_t * data, int length) {
	LOG_INF("Received data for dongle, packet %d", data[1]);
    switch(data[1]) {
        case SERVER_PACKET_SERVER_HELLO:
            process_server_hello(data, length);
        break;
        case SERVER_PACKET_SEND_TRACKERS_LIST:
            send_trackers_list();
        break;
    }
}