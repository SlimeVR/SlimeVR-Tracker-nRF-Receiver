#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_gpio.h>

#include <stdio.h>
#include <string.h>

#define DEV_ESP   DEVICE_DT_GET(DT_CHOSEN(uart_esp))

K_MSGQ_DEFINE(uart_msgq_out, 64, 10, 1);
K_MSGQ_DEFINE(uart_msgq_in, 64, 10, 1);

static const struct device *const uart_dev = DEV_ESP;

/* receive buffer used in UART ISR callback */
static char rx_buf[65];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\0') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq_in, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

static void print_uart(const struct device *dev, char *buf)
{
	for (; *buf != 0; buf++) {
		uart_poll_out(dev, *buf);
	}
	uart_poll_out(dev, 0);
}

static void uart_write_msg(uint8_t *data, int16_t len)
{
	uint8_t msg[64] = {0};
	int16_t header_len = len / 8;
	uint8_t *buf = msg + 1 + header_len;
	uint64_t *zero_flags = (uint64_t *)(msg + 1);
	uint8_t *zero_flags_buf = msg + 1;

	memcpy(buf, data, len);

	for (int i = 0; i < len; i++)
	{
		if (!*buf)
			*buf = 0x08; // doesn't matter
		else
			*zero_flags |= 1 << i;
		buf++;
	}

	for (int i = 0; i < header_len; i++)
	{
		if (!*zero_flags_buf)
			*zero_flags_buf = 0x08; // doesn't matter
		else
			*msg |= 1 << i;
		zero_flags_buf++;
	}

	*msg |= 1 << header_len; // use as flag for message length

	k_msgq_put(&uart_msgq_out, &msg, K_NO_WAIT);
}

static uint8_t *uart_read_msg(uint8_t *msg, int16_t *len)
{
	int16_t msg_len = strlen(msg);
	int16_t header_len = (msg_len - 1) / 9;
	int16_t buf_len = header_len * 8;
	uint8_t *buf = msg + 1 + header_len;
	uint64_t *zero_flags = (uint64_t *)(msg + 1);
	uint8_t *zero_flags_buf = msg + 1;

	if (msg_len != 1 + header_len + buf_len)
		return NULL;

	if (!(*msg & (1 << header_len)) || *msg >= (1 << (header_len + 1)))
		return NULL;

	for (int i = 0; i < header_len; i++)
	{
		if (!(*msg & (1 << i)))
			*zero_flags_buf = 0;
		zero_flags_buf++;
	}

	for (int i = 0; i < buf_len; i++)
	{
		if (!(*zero_flags & (1 << i)))
			*buf = 0;
		buf++;
	}

	*len = buf_len;
	return msg + 1 + header_len;
}

//|type    |description
//|TX  0x52|packet_device_list_hash (round trip ping)
//|TX  0x53|associate id and tracker address
//|TX  0x54|packet_recv_addr
//|RX  0x51|round trip response
//|RX  0x52|command

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |data                                                                                                                                  |
//|TX  0x52|stored  |crc                                |latency          |
//|TX  0x53|id      |addr                                                 |
//|TX  0x54|resv----|recv_addr                                            |
//|RX  0x51|resv----------------------------------------------------------|
//|RX  0x52|command |resv-------------------------------------------------|

// TODO: implement this

// passthrough to server

//|type    |description
//|TX   254|device list hash, round trip ping (TODO: duplicate list, but still need rtt)
//|TX   253|receiver status (TODO: send number of pairing requests found?)
//|TX   252|device pairing request, id is the id of the pairing request in case of multiple, not final tracker id (TODO: these can timeout if not sent often? can server automatically accept if the address was known?)
//|RX   255|round trip response
//|RX   254|command (TODO: include parameters?)

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |data                                                                                                                                  |
//|TX   254|stored  |crc                                |latency          |resv-------------------------------------------------------------------|
//|TX   253|status  |resv-------------------------------------------------|resv-------------------------------------------------------------------|
//|TX   252|id      |device_addr                                          |resv-------------------------------------------------------------------|
//|RX   255|resv----------------------------------------------------------|resv-------------------------------------------------------------------|
//|RX   254|command |resv-------------------------------------------------|resv-------------------------------------------------------------------|

void uart_write_packet_n(uint8_t *data, uint8_t rssi)
{
	uint8_t buf[16];

	memcpy(buf, data, 16); // all data can be passed through
	if (data[0] != 1 && data[0] != 4) // packet 1 and 4 are full precision quat and accel/mag, no room for rssi
		buf[15] = rssi;

	uart_write_msg(buf, 16);
}

int64_t round_trip_latency = 0;
int64_t start = 0;

static void packet_device_list_hash(void)
{
	uint8_t buf[8];

	buf[0] = 0x52; // prefix
	buf[1] = stored_trackers;
	// note: stored addresses use 6 bytes for the address, last two bytes are zeroes
	uint32_t crc = crc32_k_4_2_update(0x93a409eb, (uint8_t *)stored_tracker_addr, (uint16_t)stored_trackers * 8);
	memcpy(buf + 2, &crc, 4);
	int64_t latency = k_ticks_to_us_near64(round_trip_latency);
	latency = CLAMP(latency, 0, UINT16_MAX);
	*(uint16_t *)&buf[6] = (uint16_t)latency; // latency from last response in us, default to zero if not known

	uart_write_msg(buf, 8);
}

static void packet_device_addr(uint16_t id) // associate id and tracker address
{
	uint8_t buf[8];

	buf[0] = 0x53; // prefix
	buf[1] = id;
	memcpy(buf + 2, &stored_tracker_addr[id], 6);

	uart_write_msg(buf, 8);
}

static void packet_recv_addr(void)
{
	uint8_t buf[8];

	buf[0] = 0x54; // prefix
	buf[1] = 0x00;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(buf + 2, addr, 6);

	uart_write_msg(buf, 8);
}

static void scan_rx(char *buf, int16_t len)
{
	if (len != 8)
	{
		printk("Message is wrong length: %d\n", len);
		return;
	}

	switch (buf[0])
	{
	case 0x51: // round trip response
		int64_t end = k_uptime_ticks();
		round_trip_latency = end - start;
		printk("Got latency: %lld us\n", k_ticks_to_us_near64(round_trip_latency));
		break;
	case 0x52: // command
		switch (buf[1])
		{
		case 0x00: // request receiver addr
			printk("Sending receiver info\n");
			packet_recv_addr();
			break;
		case 0x01: // request device addr
			printk("Sending paired device info: %d\n", buf[1]);
			packet_device_addr(buf[1]);
			break;
		case 0x02: // reboot
//			skip_dfu();
//			sys_reboot(SYS_REBOOT_COLD);
			break;
		case 0x03: // enter pair
//			esb_reset_pair();
			break;
		case 0x04: // exit pair
//			esb_finish_pair();
			break;
		case 0x05: // clear pair
//			esb_clear();
			break;
		default:
			printk("Unknown message command: 0x%02X\n", buf[1]);
		}
		break;
	default:
		printk("Unknown message: 0x%02X\n", buf[0]);
	}
}

static void esp_reboot(bool download) {
	uint32_t run = 35;
	uint32_t boot = 36;
	nrf_gpio_cfg_output(run); // ESP EN
	nrf_gpio_cfg_output(boot); // ESP GPIO9
	nrf_gpio_pin_clear(run);
	nrf_gpio_pin_write(boot, !download);
	k_usleep(100); // min 50us hold to reset
	printk("Rebooting ESP\nDownload: %d\n", download);
	nrf_gpio_pin_set(run);
	k_msleep(5); // min 3ms hold to read strapping pins
	nrf_gpio_pin_set(boot);
}

// TODO: timer instead
static void uart_hash_thread(void)
{
	while (1)
	{
		start = k_uptime_ticks();
		packet_device_list_hash();
		k_msleep(1000);
	}
}

K_THREAD_DEFINE(uart_hash_thread_id, 1024, uart_hash_thread, NULL, NULL, NULL, 6, 0, 0);

static void uart_run_thread(void)
{
	char tx_buf[65];
	tx_buf[64] = 0;

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	uart_irq_rx_enable(uart_dev);

	while (1)
	{
		if (!k_msgq_get(&uart_msgq_out, tx_buf, K_FOREVER))
			print_uart(uart_dev, tx_buf);
	}
}

K_THREAD_DEFINE(uart_run_thread_id, 1024, uart_run_thread, NULL, NULL, NULL, 6, 0, 0);

static void uart_parse_thread(void)
{
	char rx_buf[65];
	rx_buf[64] = 0;

	while (1)
	{
		if (!k_msgq_get(&uart_msgq_in, rx_buf, K_FOREVER))
		{
			int16_t len;
//			printk("len: %2d, 0x%016llX%016llX\n", strlen(rx_buf), *(uint64_t *)(rx_buf+8), *(uint64_t *)rx_buf);
			uint8_t *buf = uart_read_msg(rx_buf, &len);
			if (buf)
				scan_rx(buf, len);
			else
				printk("Received invalid or corrupted message\n");
		}
	}
}

K_THREAD_DEFINE(uart_parse_thread_id, 1024, uart_parse_thread, NULL, NULL, NULL, 6, 0, 0);
