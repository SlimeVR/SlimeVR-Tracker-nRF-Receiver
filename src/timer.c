#include "globals.h"
#include "esb.h"

#include <nrfx_timer.h>

const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);
uint16_t led_clock = 0;

LOG_MODULE_REGISTER(timer, 4);

void timer_handler(nrf_timer_event_t event_type, void *p_context) {
	if (event_type == NRF_TIMER_EVENT_COMPARE0) {
		//esb_write_sync(led_clock);
		esb_start_tx();
	} else if (event_type == NRF_TIMER_EVENT_COMPARE1) {
		esb_stop_rx();
		esb_disable();
		esb_initialize(true);
		esb_write_sync(led_clock);
	} else if (event_type == NRF_TIMER_EVENT_COMPARE2) {
		esb_disable();
		esb_initialize(false);
		esb_start_rx();
		led_clock++;
		led_clock%=17*600/3;
	}
}

void timer_init(void) {
    //nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	//timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    //timer_cfg.mode = NRF_TIMER_MODE_TIMER;
    //timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
    //timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
    //timer_cfg.p_context = NULL;
	nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
    uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 3);
    nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true); // timeslot to send sync
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks * 20 / 21, true); // switch to tx
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, ticks * 1 / 21, true); // switch to rx
    nrfx_timer_enable(&m_timer);
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
}
