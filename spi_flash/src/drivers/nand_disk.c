/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SDMMC disk driver using zephyr SD subsystem
 */
#define DT_DRV_COMPAT zephyr_sdmmc_disk

#include <zephyr/drivers/disk.h>
#include "nand_flash.h"


enum sd_status {
	SD_UNINIT,
	SD_ERROR,
	SD_OK,
};

struct sdmmc_config {
	const struct device *host_controller;
};

struct sdmmc_data {
	struct sd_card card;
	enum sd_status status;
	char *name;
};


static int disk_nand_access_init(struct disk_info *disk)
{
	
	return 0;
}

static int nand_access_status(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	const struct sdmmc_config *cfg = dev->config;
	struct sdmmc_data *data = dev->data;

	if (!sd_is_card_present(cfg->host_controller)) {
		return DISK_STATUS_NOMEDIA;
	}
	if (data->status == SD_OK) {
		return DISK_STATUS_OK;
	} else {
		return DISK_STATUS_UNINIT;
	}
}

static int disk_nand_access_read(struct disk_info *disk, uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;

	return sdmmc_read_blocks(&data->card, buf, sector, count);
}

static int disk_nand_access_write(struct disk_info *disk, const uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;

	return sdmmc_write_blocks(&data->card, buf, sector, count);
}

static int disk_nand_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;

	return sdmmc_ioctl(&data->card, cmd, buf);
}

static const struct disk_operations sdmmc_disk_ops = {
	.init = disk_nand_access_init,
	.status = disk_nand_access_status,
	.read = disk_nand_access_read,
	.write = disk_nand_access_write,
	.ioctl = disk_nand_access_ioctl,
};

static struct disk_info sdmmc_disk = {
	.ops = &sdmmc_disk_ops,
};

static int disk_sdmmc_init(const struct device *dev)
{
	struct sdmmc_data *data = dev->data;

	data->status = SD_UNINIT;
	sdmmc_disk.dev = dev;
	sdmmc_disk.name = data->name;

	return disk_access_register(&sdmmc_disk);
}

#define DISK_ACCESS_SDMMC_INIT(n)						\
	static const struct sdmmc_config sdmmc_config_##n = {			\
		.host_controller = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};									\
										\
	static struct sdmmc_data sdmmc_data_##n = {				\
		.name = CONFIG_SDMMC_VOLUME_NAME,				\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			&disk_sdmmc_init,					\
			NULL,							\
			&sdmmc_data_##n,					\
			&sdmmc_config_##n,					\
			POST_KERNEL,						\
			CONFIG_SD_INIT_PRIORITY,				\
			NULL);

DT_INST_FOREACH_STATUS_OKAY(DISK_ACCESS_SDMMC_INIT)
