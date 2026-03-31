#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <hal/nrf_gpio.h>

#define DEV_SOURCE DEVICE_DT_GET(DT_CHOSEN(uart_host))
#define DEV_PEER   DEVICE_DT_GET(DT_CHOSEN(uart_passthrough))

struct uart_config uart_cfg_peer = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

static void uart_bridge_thread(void)
{
	uint32_t run = 35;
	uint32_t boot = 36;
	nrf_gpio_cfg_output(run); // ESP EN
	nrf_gpio_cfg_output(boot); // ESP GPIO9
	nrf_gpio_pin_set(boot);
	nrf_gpio_pin_set(run);
	uint32_t baudrate;
	uint32_t last_rts = 0;
	uint32_t last_dtr = 0;
	while (1)
	{
		uint32_t rts;
		uint32_t dtr;
		uart_line_ctrl_get(DEV_SOURCE, UART_LINE_CTRL_BAUD_RATE, &baudrate);
		uart_line_ctrl_get(DEV_SOURCE, UART_LINE_CTRL_RTS, &rts);
		uart_line_ctrl_get(DEV_SOURCE, UART_LINE_CTRL_DTR, &dtr);
		if (uart_cfg_peer.baudrate != baudrate)
		{
			uart_cfg_peer.baudrate = baudrate;
			uart_configure(DEV_PEER, &uart_cfg_peer);
			printk("Set UART0 to baudrate: %u\n", uart_cfg_peer.baudrate);
		}
		if (last_rts != rts)
		{
			nrf_gpio_pin_write(run, last_rts);
//			if (last_rts)
//				printk("Clear RTS, exit reset\n");
//			else
//				printk("Set RTS, reset SOC\n");
			if (last_rts)
				printk("Rebooting ESP\nDownload: %d\n", last_dtr);
			last_rts = rts;
		}
		if (last_dtr != dtr)
		{
			nrf_gpio_pin_write(boot, last_dtr);
//			if (last_dtr)
//				printk("Clear DTR, clear download flag\n");
//			else
//				printk("Set DTR, set download mode flag\n");
			last_dtr = dtr;
		}
		k_usleep(1);
	}
}

K_THREAD_DEFINE(uart_bridge_thread_id, 1024, uart_bridge_thread, NULL, NULL, NULL, 6, 0, 0);
