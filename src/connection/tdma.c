/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Eiren Rain & SlimeVR Contributors

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
#include "tdma.h"

uint8_t tdma_tracker_to_window[MAX_TRACKERS] = {TDMA_WRONG_WINDOW};
uint8_t tdma_windows[TDMA_MAX_TRACKERS] = {TDMA_WRONG_WINDOW};

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

void tdma_init() {
     for(int i = 0; i < TDMA_MAX_TRACKERS; ++i) {
        tdma_windows[i] = TDMA_WRONG_WINDOW;
     }
     for(int i = 0; i < MAX_TRACKERS; ++i) {
        tdma_tracker_to_window[i] = TDMA_WRONG_WINDOW;
     }
}

uint32_t tdma_get_timer() {
    return k_cycle_get_32() & TDMA_TIMER_MASK;
}

uint32_t tdma_get_slot(uint32_t timer) {
    return timer >> TDMA_SLOT_SHIFT;
}

uint8_t tdma_get_window(uint32_t slot) {
    return (slot - 24) % 10;
}

bool tdma_is_dongle_window(uint32_t slot) {
    return slot < 24;
}

uint8_t tdma_get_tracker_window(uint8_t tracker_id) {
    return tdma_tracker_to_window[tracker_id];
}

uint8_t tdma_get_or_allocate_tracker_window(uint8_t tracker_id) {
    uint8_t window = tdma_tracker_to_window[tracker_id];
    if(window == TDMA_WRONG_WINDOW) {
        // No window is allocated to the tracker, try to find an empty one and allocate
        for(int i = 0; i < TDMA_MAX_TRACKERS; ++i) {
            if(tdma_windows[i] == TDMA_WRONG_WINDOW) {
                window = i;
                tdma_windows[i] = tracker_id;
                tdma_tracker_to_window[tracker_id] = window;
                LOG_INF("Allocated window %d for tracker id %d", window, tracker_id);
                break;
            }
        }
    }
    return window;
}

// Tracker communicated with the dongle, record last time
// So we can de-allocated windows for trackers that aren't connected anymore
uint8_t tdma_touch_tracker(uint8_t tracker_id) {

}