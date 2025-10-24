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
#include <stdint.h>
#include "globals.h"

#define TDMA_TIMER_SIZE 32768
#define TDMA_WINDOW_SIZE 16
#define TDMA_WINDOWS_COUNT 2048
#define TDMA_WINDOWS_SHIFT 11
#define TDMA_MAX_TRACKERS 32

uint8_t tdma_tracker_slots = 8;
uint8_t tdma_window_allocations = 256;
int8_t tdma_tracker_id_to_slot[MAX_TRACKERS] = {-1};
uint8_t tdma_next_tracker_slot = 0;

bool tdma_adjust_window_size(uint8_t trackers_amount) {
    if(trackers_amount <= 8)
        tdma_tracker_slots = 8;
    else if(trackers_amount <= 16)
        tdma_tracker_slots = 16;
    else
        tdma_tracker_slots = 32;
    tdma_window_allocations = TDMA_WINDOWS_COUNT / tdma_tracker_slots;
    return trackers_amount <= tdma_tracker_slots;
}

uint16_t tdma_get_window_from_timer(uint16_t timer) {
    return timer >> TDMA_WINDOWS_SHIFT;
}

uint8_t tdma_get_tracker_slot_from_timer(uint16_t timer) {
    return tdma_get_window_from_timer(timer) % tdma_tracker_slots;
}

uint16_t tdma_get_tracker_slot_error(uint16_t timer, uint8_t tracker_slot) {
    uint8_t correct_tracker = tdma_get_tracker_slot_from_timer(timer);
    if(correct_tracker == tracker_slot)
        return 0;
    while(correct_tracker < tracker_slot) // We always go to the future
        correct_tracker += tdma_tracker_slots;
    return timer - (correct_tracker - tracker_slot) * TDMA_WINDOW_SIZE;
}

int8_t tdma_tracker_slot_from_id(uint8_t tracker_id) {
    return tdma_tracker_id_to_slot[tracker_id];
}

int8_t tdma_insert_tracker(uint8_t tracker_id) {
    int8_t current_slot = tdma_tracker_slot_from_id(tracker_id);
    if(current_slot >= 0)
        return current_slot;
    if(!tdma_adjust_window_size(tdma_next_tracker_slot + 1))
        return -1;
    tdma_tracker_id_to_slot[tracker_id] = tdma_next_tracker_slot;
    return tdma_next_tracker_slot++;
}

bool tdma_is_dongle_order(uint16_t timer) {
    return tdma_get_window_from_timer(timer) < tdma_tracker_slots;
}
