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

#pragma once

#include "zephyr/toolchain.h"

void hid_dongle_message(uint8_t * data, int length);

// TODO: using packed attribute is maybe not super safe, could use distinct uint8_t type in structs instead and cast from enum
enum server_type {
    SERVER_UNKNOWN = 0,
    SERVER_NORMAL = 1,
    SERVER_ASTERTRACK = 2
} __packed;

enum esb_protocol_version {
    P_VERSION_OLD = 0,
    P_VERSION_LEGACY = 2,
    P_VERSION_TRANSITIONAL = 3,
    P_VERSION_MODERN = 4
} __packed;

typedef struct packet_server_hello {
    uint8_t seq;
    uint8_t packet_id;
    enum esb_protocol_version protocol_version;
    enum server_type server_type;
    unsigned int : 7;
    unsigned int flag_send_all : 1;
    uint64_t server_time;
} __packed packet_server_hello_t;

enum dongle_info_data_type {
    DATA_BASIC_INFO = 1,
    DATA_MODEL = 2,
    DATA_MANUFACTURER = 3,
    DATA_FIRMWARE_VERSION = 4,
    DATA_FIRMWARE_DATE = 5,
    DATA_CUSTOM_HW_TYPE = 6,
    DATA_CHANNEL_SETTINGS = 7,
} __packed;

typedef struct packet_dongle_into {
    uint8_t seq;
    uint8_t packet_id;
    enum dongle_info_data_type data_type;
    uint8_t data_length;
    uint8_t data_array[59];
} __packed packet_dongle_into_t;

enum dongle_hardware_type {
    HARDWARE_UNKNOWN = 0,
    HARDWARE_DEV_BUTTERFLY_DONGLE = 1,
    HARDWARE_BUTTERFLY_DONGLE = 2,
    HARDWARE_HOLYIOT_DONGLE = 3,
    HARDWARE_CUSTOM = 255
} __packed;

typedef struct packet_dongle_info_basic {
    uint64_t hwid;
    uint8_t dongle_class;
    uint8_t dongle_type;
    enum dongle_hardware_type dongle_hardware_type;
    uint8_t dongle_hardware_revision;
    enum esb_protocol_version protocol_version;
} __packed packet_dongle_info_basic_t;
