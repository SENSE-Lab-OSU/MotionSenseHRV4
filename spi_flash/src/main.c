/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/storage/disk_access.h>
#include "drivers/nand_disk.h"
#include "drivers/spi_nand.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/flash/nrf_qspi_nor.h>
#include <stdio.h>
#include <string.h>
void test_cmd();

#if defined(CONFIG_BOARD_ADAFRUIT_FEATHER_STM32F405)
#define SPI_FLASH_TEST_REGION_OFFSET 0xf000
#elif defined(CONFIG_BOARD_ARTY_A7_ARM_DESIGNSTART_M1) || \
	defined(CONFIG_BOARD_ARTY_A7_ARM_DESIGNSTART_M3)
/* The FPGA bitstream is stored in the lower 536 sectors of the flash. */
#define SPI_FLASH_TEST_REGION_OFFSET \
	DT_REG_SIZE(DT_NODE_BY_FIXED_PARTITION_LABEL(fpga_bitstream))
#elif defined(CONFIG_BOARD_NPCX9M6F_EVB) || \
	defined(CONFIG_BOARD_NPCX7M6FB_EVB)
#define SPI_FLASH_TEST_REGION_OFFSET 0x7F000
#else
#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
#endif
#define SPI_FLASH_SECTOR_SIZE        4096

uint8_t expected[4096];

int storage_main(void);

void main(void){
	printf("Start\n");
	k_sleep(K_SECONDS(2));
	main2(true);
	k_sleep(K_SECONDS(2));
	storage_main();
	
} 

void main2(bool chip_erase)
{
	//test_cmd();
	
	for (int i = 0; i < sizeof(expected); i++){
		expected[i] = i % 9;
	}
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	const struct device *flash_dev;
	int rc;

	const struct device* test_qspi_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	printk("got first device");
	
	flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	const struct device* spi_dev; //= DEVICE_DT_GET(DT_NODELABEL(spi4));
	spi_dev = device_get_binding("spi@9000");
	//DEVICE_DT_GET(DT_NODELABEL(spi0));


	//flash_dev = NULL;
	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		return;
	}


	const struct disk_operations* disk_api = (const struct disk_operations*)spi_dev->api;
 
	printf("\n%s SPI flash testing\n", flash_dev->name);
	printf("==========================\n");
	k_sleep(K_MSEC(500));

	
	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase\n");
	k_sleep(K_MSEC(500));
	
	//detect_bad_blocks(flash_dev);
	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	if (chip_erase){
	rc = spi_nand_chip_erase(flash_dev);
	}
	//rc = spi_nand_block_erase(flash_dev, 65);
	//rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET,
	//		 SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write\n");

	set_die(flash_dev, 0);

	printf("Attempting to write %zu bytes\n", len);
	//rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	
	
	const char* disk_name = "SD";
	
	/*
	for (int x = 20; x < 65; x++){
	disk_access_write(disk_name, expected, x, 1);
	}
	*/
	
	
	//rc = disk_api->write(&sdmmc_disk, expected, 4, 0);
	//rc = spi_nand_page_write(flash_dev, 4, expected, len);
	//rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printf("Flash write failed! %d\n", rc);
		return;
	}

	// 4 gigabit is 536870912 bytes / 4096 = 131072 pages (131071 is last address)

	memset(buf, 0, len);
	rc = disk_access_read(disk_name, buf, 1, 1);
	//rc = spi_nand_page_read(flash_dev, 4, buf); 
	//rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return;
	}
	bool print_results = true;
	const uint8_t *wp = expected;
	const uint8_t *rp = buf;
	const uint8_t *rpe = rp + 100;

	if (memcmp(expected, buf, len) == 0) {
		printf("Data read matches data written. Good!!\n");
		while (rp < rpe && print_results) {
			printf("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	} else {

		printf("Data read does not match data written!!\n");
		while (rp < rpe && print_results) {
			printf("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
		
	}
}
