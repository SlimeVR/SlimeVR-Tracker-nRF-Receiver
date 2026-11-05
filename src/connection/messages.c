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
static const enum esb_protocol_version target_protocol_version = P_VERSION_TRANSITIONAL;
static enum esb_protocol_version protocol_version = P_VERSION_LEGACY;
static enum server_type type_of_server = SERVER_NORMAL;

void send_dongle_info() {
    packet_dongle_info_t dongle_info;
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
    basic_info.protocol_version = target_protocol_version;
    memcpy(dongle_info.data_array, &basic_info, sizeof(packet_dongle_info_basic_t));
    hid_write_packet_n((uint8_t *) &dongle_info, sizeof(packet_dongle_info_t));
    // TODO Send other info
}

enum esb_protocol_version current_esb_protocol_version() {
    return protocol_version;
}

enum server_type get_server_type() {
    return type_of_server;
}

void msg_send_tracker_info(uint8_t tracker_id) {
    uint64_t tracker_addr = esb_get_tracker_addr(tracker_id);
    packet_trackers_list_t tracker_packet;
    tracker_packet.packet_id = SERVER_PACKET_TRACKERS_LIST;
    tracker_packet.sensors_num = 1; // TODO : Tracker sensors qtty and send it to the server maybe probably
    tracker_packet.tracker_hwid = tracker_addr;
    tracker_packet.tracker_id = tracker_id;
    // TODO Better status check
    tracker_packet.tracker_status = tracker_addr != 0 && tdma_get_tracker_window(tracker_id) != TDMA_WRONG_WINDOW ? 1 : 0;
    hid_write_packet_n((uint8_t *) &tracker_packet, sizeof(packet_trackers_list_t));
}

void msg_send_trackers_list(bool send_all) {
    for(int i = 0; i < MAX_TRACKERS; ++i) {
        uint64_t tracker_addr = esb_get_tracker_addr(i);
        if(send_all || tracker_addr != 0) {
            msg_send_tracker_info(i);
        }
    }
}

void process_server_hello(uint8_t * data, int length) {
    if(length < sizeof(packet_server_hello_t)) {
        LOG_WRN("Too short SERVER HELLO (%d) packet: %d bytes", SERVER_PACKET_SERVER_HELLO, length);
        return;
    }
    packet_server_hello_t server_hello = *(packet_server_hello_t * ) &data[0];
    LOG_INF("Server hello: protocol %d, server type: %d, flags: %d, time %llu", server_hello.protocol_version, server_hello.server_type, data[4], server_hello.server_time);
    if(server_hello.flag_send_all || !sent_data) {
        protocol_version = MIN(target_protocol_version, server_hello.protocol_version);
        type_of_server = server_hello.server_type;
        sent_data = true;
        send_dongle_info();
        msg_send_trackers_list(false);
    }
    // TODO Time sync
}

void process_clear_pairing(uint8_t * data, int length) {
    if(length < 3) {
        LOG_WRN("Too short CLEAR PAIRING DATA (%d) packet: %d bytes", SERVER_PACKET_CLEAR_PAIRING, length);
        return;
    }
    uint8_t to_clear = data[2];
    if(to_clear == 255) {
        esb_clear();
    } else if(to_clear < 201) {
        esb_remove_pair(to_clear);
    }
}

void hid_dongle_message(uint8_t * data, int length) {
	LOG_INF("Received data for dongle, packet %d", data[1]);
    switch(data[1]) {
        case SERVER_PACKET_SERVER_HELLO:
            process_server_hello(data, length);
        break;
        case SERVER_PACKET_SEND_TRACKERS_LIST:
            msg_send_trackers_list(false);
        break;
        case SERVER_PACKET_CLEAR_PAIRING:
            process_clear_pairing(data, length);
        break;
    }
}