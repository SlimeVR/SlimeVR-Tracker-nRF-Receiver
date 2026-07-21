#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include "message.h"

#define QUEUE_SIZE 5

struct k_msgq tracker_queues[MAX_TRACKERS];

static char __aligned(4) queue_buffers[MAX_TRACKERS][QUEUE_SIZE * sizeof(hid_to_esb)];

void init_tracker_message_queues(void)
{
    for (int i = 0; i < MAX_TRACKERS; i++)
    {
        k_msgq_init(&tracker_queues[i], queue_buffers[i], sizeof(hid_to_esb), QUEUE_SIZE);
    }
}
