#include "globals.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>

#include "system.h"

static struct nvs_fs fs;

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

static bool nvs_init = false;

static int sys_nvs_init(void) {
	if (nvs_init) {
		return 0;
	}
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // starting at NVS_PARTITION_OFFSET
	if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info))
	{
		LOG_ERR("Failed to get page info");
		return 1;
	}
	fs.sector_size = info.size; // sector_size equal to the pagesize
	fs.sector_count = 4U; // 4 sectors
	int err = nvs_mount(&fs);
	if (err == -EDEADLK)
	{
		LOG_WRN("All sectors closed, erasing all sectors...");
		err = flash_flatten(fs.flash_device, fs.offset, fs.sector_size * fs.sector_count);
		if (!err)
			err = nvs_mount(&fs);
	}
	if (err)
	{
		LOG_ERR("Failed to mount NVS");
		return 1;
	}
	nvs_init = true;
	return 0;
}

SYS_INIT(sys_nvs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// TODO: switch back to retained?
uint8_t reboot_counter_read(void) {
	uint8_t reboot_counter;
	nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	return reboot_counter;
}

void reboot_counter_write(uint8_t reboot_counter) {
	nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
}

// retained not implemented
void sys_write(uint16_t id, void* retained_ptr, const void* data, size_t len) {
	sys_nvs_init();
	nvs_write(&fs, id, data, len);
}

// reading from nvs
void sys_read(uint16_t id, void* data, size_t len) {
	sys_nvs_init();
	nvs_read(&fs, id, data, len);
}
