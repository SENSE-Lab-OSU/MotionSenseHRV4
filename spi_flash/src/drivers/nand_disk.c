/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SDMMC disk driver using zephyr SD subsystem
 */

#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include "spi_nand.h"
#include "nand_disk.h"

#define DT_DRV_COMPAT senselab_nanddisk

LOG_MODULE_REGISTER(nand_disk, 4);

enum sd_status {
	SD_UNINIT,
	SD_ERROR,
	SD_OK,
};

struct sdmmc_config {
	//const struct device *host_controller;
};

struct sdmmc_data {
	//struct sd_card card;
	enum sd_status status;
	char *name;
};


// Config sector monitoring
int sector_write_list[500] = { 0 };
int unique_sectors_written = 0;

char sector_1_buffer[4096];
char sector_2_buffer[4096];

static int duplicate_sector_access(int sector_num){
	for (int i = 0; i < unique_sectors_written; i++){
		if (sector_write_list[i] == sector_num){
			LOG_WRN("error: attempted duplicate write for sector %i", sector_num);
			return -1;
		}
	}
	sector_write_list[unique_sectors_written] = sector_num;
	unique_sectors_written++;
	return 0;
}


static int disk_nand_access_init(struct disk_info *disk)
{
	const struct device* dev = disk->dev;
	int sucess = spi_init(dev);
	return sucess;
}


static int disk_acess_init2(struct disk_info *disk){
	return 0;
} 

static int disk_nand_access_status(struct disk_info *disk)
{
	LOG_INF("Accessing Status");
	const struct device* dev = disk->dev;
	
	/*const struct sdmmc_config* cfg = dev->config;
	struct sdmmc_data *data = dev->data;

	if (!sd_is_card_present(cfg->host_controller)) {
		return DISK_STATUS_NOMEDIA;
	}
	if (data->status == SD_OK) {
		return DISK_STATUS_OK;
	} else {
		return DISK_STATUS_UNINIT;
	}
	*/
	//uint8_t status = spi_nor_rdsr(dev);
	return DISK_STATUS_OK;

}

static int disk_nand_access_read(struct disk_info* disk, uint8_t *buf,
				 uint32_t sector, uint32_t count)
{


	LOG_DBG("performing disk read at sector %i", sector);
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;
	
	if (sector == 1){
		memcpy(buf, sector_1_buffer, 4096);
		return 0;
	}
	else if (sector == 2){
		memcpy(buf, sector_2_buffer, 4096);
		return 0;
	}

	if (count > 1){
	LOG_WRN("count: %i", count);
	}

	off_t addr;
	int ret = 0;
	
	for (int x = 0; x < count; x++) {
	addr = convert_page_to_address(sector);
	ret = spi_nand_page_read(dev, addr, buf);
	}
	
	//lol
	return ret; //sdmmc_read_blocks(&data->card, buf, sector, count);
}

static int disk_nand_access_write(struct disk_info *disk, const uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	LOG_DBG("performing disk write at sector %i", sector);
	if (count > 1){
	LOG_WRN("count: %i", count);
	}
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;
	int ret;
	off_t addr;
	
	// Do we know what count means?
	if (sector == 1){
		memcpy(sector_1_buffer, buf, 4096);
		return 0;
	}
	else if (sector == 2){
		memcpy(sector_2_buffer, buf, 4096);
		return 0;
	}else {
	duplicate_sector_access(sector);
	}

	for (int x = 0; x < count; x++){
		addr = convert_page_to_address(sector+x);
		ret = spi_nand_page_write(dev, addr, buf, 4096);
		
	}
	
	
	return ret; //sdmmc_write_blocks(&data->card, buf, sector, count);
}

static int disk_nand_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	LOG_WRN("Acessing ioctl with cmd %d", cmd);
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;

    switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		(*(uint32_t *)buf) = 5000;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		(*(uint32_t *)buf) = dev_page_size(dev); 
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		(*(uint32_t *)buf) = dev_page_size(dev)*64;
		break;
	case DISK_IOCTL_CTRL_SYNC:
		/* Ensure card is not busy with data write.
		 * Note that SD stack does not support enabling caching, so
		 * cache flush is not required here
		 */
		return 0; //spi_nor_wait_until_ready(dev);
	default:
		return -ENOTSUP;
	}
	return 0;


	return 0; //sdmmc_ioctl(&data->card, cmd, buf);
}

static const struct disk_operations sdmmc_disk_ops = {
	.init = disk_acess_init2,
	.status = disk_nand_access_status,
	.read = disk_nand_access_read,
	.write = disk_nand_access_write,
	.ioctl = disk_nand_access_ioctl,
};

struct disk_info sdmmc_disk = {
	.ops = &sdmmc_disk_ops,
};

#define CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE 4096

#define LAYOUT_PAGES_COUNT 1000

BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size),
	     "jedec,spi-nor size required for non-runtime SFDP page layout");

#if defined(CONFIG_FLASH_PAGE_LAYOUT)


BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size),
	     "jedec,spi-nor size required for non-runtime SFDP page layout");


#define INST_0_BYTES (DT_INST_PROP(0, size) / 8)

BUILD_ASSERT(SPI_NOR_IS_SECTOR_ALIGNED(CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE),
	     "SPI_NOR_FLASH_LAYOUT_PAGE_SIZE must be multiple of 4096");


#define LAYOUT_PAGES_COUNT (INST_0_BYTES / CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE)

BUILD_ASSERT((CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE * LAYOUT_PAGES_COUNT)
	     == INST_0_BYTES,
	     "SPI_NOR_FLASH_LAYOUT_PAGE_SIZE incompatible with flash size");

#endif 



static const struct spi_flash_config spi_flash_config_0 = 
{
	
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8),
				    0),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif

#if !defined(CONFIG_SPI_NOR_SFDP_RUNTIME)

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.layout = {
		.pages_count = LAYOUT_PAGES_COUNT,
		.pages_size = CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE,
	},
#undef LAYOUT_PAGES_COUNT
#endif 

	.flash_size = DT_INST_PROP(0, size) / 8,
	.jedec_id = DT_INST_PROP(0, jedec_id),

#if DT_INST_NODE_HAS_PROP(0, has_lock)
	.has_lock = DT_INST_PROP(0, has_lock),
#endif
#if defined(CONFIG_SPI_NOR_SFDP_MINIMAL)		\
	&& DT_INST_NODE_HAS_PROP(0, enter_4byte_addr)
	.enter_4byte_addr = DT_INST_PROP(0, enter_4byte_addr),
#endif
#ifdef CONFIG_SPI_NOR_SFDP_DEVICETREE
	.bfp_len = sizeof(bfp_data_0) / 4,
	.bfp = (const struct jesd216_bfp *)bfp_data_0,
	
#endif 

#endif 

};

static struct spi_nor_data spi_nor_data_0;




static int disk_sdmmc_init(const struct device *dev)
{
	//struct sdmmc_data* data = dev->data;
	//data->status = SD_UNINIT;

	//spi_nor_data* nand_data = dev->data;
	LOG_INF("Initializing disk Registration"); 

	sdmmc_disk.dev = dev;
	sdmmc_disk.name = "SD";//dev->name;
	disk_nand_access_init(&sdmmc_disk);
	return disk_access_register(&sdmmc_disk);
}

/*
#define DISK_ACCESS_SDMMC_INIT(n)						\
	static const struct sdmmc_config sdmmc_config_##n = {			\
		.host_controller = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};									\
										\
	static struct sdmmc_data sdmmc_data_##n = {				\
		.name = CONFIG_SDMMC_VOLUME_NAME,				\
	};									\
										\

*/


	DEVICE_DT_INST_DEFINE(0,						
			&disk_sdmmc_init,					
			NULL,							
			&spi_nor_data_0,					
			&spi_flash_config_0,					
			POST_KERNEL,						
			80,				
			NULL);

//DT_INST_FOREACH_STATUS_OKAY(DISK_ACCESS_SDMMC_INIT)
