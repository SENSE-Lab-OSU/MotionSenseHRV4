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
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, 3);

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

#define LED0_NODE DT_ALIAS(led0)

#define LED_PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define LED_FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

#define POWER_NODE DT_ALIAS(led2)
#define POWER_PIN DT_GPIO_PIN(POWER_NODE, gpios)
#define POWER_FLAGS DT_GPIO_FLAGS(POWER_NODE, gpios)


int flash_erase_test(const struct device* flash_dev, bool chip_erase){

	int rc;
	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	
	
	//detect_bad_blocks(flash_dev);
	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	
	if (chip_erase){
	#ifdef CONFIG_DISK_DRIVER_RAW_NAND
	//rc = spi_nand_whole_chip_erase(flash_dev);
	rc = spi_nand_multi_chip_erase(flash_dev);
	//rc = spi_nand_block_erase(flash_dev, 64);
	#endif
	if (rc != 0) {
		printk("Flash erase failed! %d\n", rc);
	} else {
		printk("Flash erase succeeded!\n");
	}
	}
	return rc;
}

#ifdef CONFIG_DISK_DRIVER_RAW_NAND
int flash_simple_read_erased_test(const struct device* flash_dev){
	int rc;

	return rc;
}	

int flash_simple_write_read_test(const struct device* flash_dev, bool write, bool read_out_whole_page, int sector){
	uint8_t expected[4096];
	int rc;
	for (int i = 0; i < sizeof(expected); i++){
		expected[i] = i % 9;
	}
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];



	
	printk("Attempting to write %zu bytes\n", len);
				
		
	
		if (write){
			rc = spi_nand_page_write(flash_dev, sector, expected, len);
		}
	
	if (rc != 0) {
		printk("Flash write failed! %d\n", rc);
	}
	const char* disk_name = "SD";
	// 4 gigabit is 536870912 bytes / 4096 = 131072 pages (131071 is last address)

	memset(buf, 0, len);
	//set_flash(flash_dev, 1);
	rc = spi_nand_page_read(flash_dev, sector, buf);
	if (rc != 0) {
		printk("Flash read failed! %d\n", rc);
	}
	bool print_results = true;
	const uint8_t *wp = expected;
	const uint8_t *rp = buf;
	
	int print_amount = 4096; //? 100: read_out_whole_page;
	const uint8_t *rpe = rp + print_amount;

	if (memcmp(expected, buf, len) == 0) {
		printk("Data read matches data written. Good!!\n");
		while (rp < rpe && print_results) {
			if (*rp != 0xff){
			printk("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			}
			++rp;
			++wp;
		}
	} else {

		printk("Data read does not match data written!!\n");
		while (rp < rpe && print_results) {
			if (*rp != 0xff){
			printk("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			}
			++rp;
			++wp;
		}
		
	}


}

void flash_multi_write_read_test(const struct device* flash_dev, bool write, int sect_num){

	uint8_t expected[4096];
	for (int i = 0; i < sizeof(expected); i++){
		expected[i] = i % 9;
	}
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	
	int rc = 0;

	for (int sector_num = sect_num; sector_num < sect_num + 2; sector_num++){
		printk("Attempting to read/write %zu bytes to %d \n", len, sector_num);
	//rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
		
		
		const char* disk_name = "SD";
	
		if (write){
	
			disk_access_write(disk_name, expected, sector_num, 1);
			//disk_access_write(disk_name, expected, sector_num, 1);
		}
	//rc = disk_api->write(&sdmmc_disk, expected, 4, 0);
	//rc = spi_nand_page_write(flash_dev, 4, expected, len);
	//rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printk("Flash write failed! %d\n", rc);
	}
	
	// 4 gigabit is 536870912 bytes / 4096 = 131072 pages (131071 is last address)

	memset(buf, 0, len);
	//set_flash(flash_dev, 1);
	rc = disk_access_read(disk_name, buf, sector_num, 1);
	set_flash(flash_dev, 0);
	//rc = spi_nand_page_read(flash_dev, 4, buf); 
	//rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printk("Flash read failed! %d\n", rc);
		
	}
	bool print_results = true;
	const uint8_t *wp = expected;
	const uint8_t *rp = buf;
	const uint8_t *rpe = rp + 50;

	if (memcmp(expected, buf, len) == 0) {
		printk("Data read matches data written. Good!!\n");
		while (rp < rpe && print_results) {
			printk("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	} else {

		printk("Data read does not match data written!!\n");
		while (rp < rpe && print_results) {
			printk("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
		
	}

	}
	
	k_sleep(K_MSEC(500));
}

void flash_testing_and_erase(bool chip_erase, bool write)
{
	//test_cmd();
	
	const struct device *flash_dev;
	const struct device* test_qspi_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	printk("got first device\n");
	
	flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	const struct device* spi_dev; //= DEVICE_DT_GET(DT_NODELABEL(spi4));
	spi_dev = device_get_binding("spi@9000");
	

	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		return;
	}


	const struct disk_operations* disk_api = (const struct disk_operations*)spi_dev->api;
 
	printk("\n%s SPI flash testing\n", flash_dev->name);
	printk("==========================\n");
	k_sleep(K_MSEC(500));

	printk("\nTest 1: Flash erase\n");
	k_sleep(K_MSEC(500));
	flash_erase_test(flash_dev, chip_erase);
	

	printk("\nTest 2: Flash write\n");
	//flash_simple_write_read_test(flash_dev, write, true, 27141);
	
	
	flash_multi_write_read_test(flash_dev, write, 72);
	
	flash_multi_write_read_test(flash_dev, write, 400);
	flash_multi_write_read_test(flash_dev, write, 2061);
	flash_multi_write_read_test(flash_dev, write, 27141);
	flash_multi_write_read_test(flash_dev, write, 27295);
	flash_multi_write_read_test(flash_dev, write, 131071);
	flash_multi_write_read_test(flash_dev, write, 131072);
	flash_multi_write_read_test(flash_dev, write, 131075*2);
	flash_multi_write_read_test(flash_dev, write, (131075*3) + 50);
	
	printk("done with all tests!\n");
}
#endif

void main(void){
	LOG_INF("Start\n");
	const struct device* gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	int ret = gpio_pin_configure(gpio_dev, LED_PIN, GPIO_OUTPUT_ACTIVE |LED_FLAGS);
	ret = gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), POWER_PIN, GPIO_OUTPUT_ACTIVE | POWER_FLAGS);
	ret = usb_enable(NULL);
	k_sleep(K_SECONDS(2));
	flash_testing_and_erase(false, false);
	
	
	
	k_sleep(K_SECONDS(8));
	storage_main();
	usb_disable();
	usb_enable(NULL);
	
	
	ret = gpio_pin_set(gpio_dev, LED_PIN, 0);
	LOG_INF("end of main");
	
    
	
} 

