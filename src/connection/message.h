#ifndef MESSAGE_H
#define MESSAGE_H

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include "globals.h"

typedef struct
{
    uint64_t tracker_id;
    uint8_t command;
} hid_to_esb;

extern struct k_msgq tracker_queues[MAX_TRACKERS];

void init_tracker_message_queues(void);

#endif