/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <stdio.h>
#include <zephyr/random/rand32.h>
#include "filesystem/zephyrfilesystem.h"

#define RAND_MAX 64
LOG_MODULE_REGISTER(main);

int64_t start_time;

void start_timer(){
	start_time = k_uptime_get();
}


int64_t stop_timer(){
	int64_t length = k_uptime_get() - start_time;
	start_time = 0;
	return length;
}


void main(void)
{

	int test_data[1000] = {1, 2, 4, 5, 6, };
	int ret;
	for (int counter = 0; counter < sizeof(test_data)/sizeof(int); counter++){
		test_data[counter] = sys_rand32_get() % 10000;
	}


	setup_disk();
	printk("disk setup complete!\n");
	k_sleep(K_SECONDS(2));
	//create_test_files();
	write_to_file(test_data, sizeof(test_data));
	int k = 0;
	start_timer();
	for (int x = 0; x < 10; x++){
		write_to_file(test_data, sizeof(test_data));
	}

	
	uint64_t total_time = stop_timer();
	printk("time: %llu \n", total_time);
	close_all_files();
	ret = usb_enable(NULL);
	
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	printk("%i \n", k);
	LOG_INF("Main Mass Application\n");
	LOG_INF("The device is put in USB mass storage mode.\n");
}



