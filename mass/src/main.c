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
#include "filesystem/zephyrfilesystem.h"

LOG_MODULE_REGISTER(main);

int64_t start_time;

void start_timer(){
	start_time = k_uptime_get();
}


int64_t stop_timer(){
	int64_t length = k_uptime_get - start_time;
	start_time = 0;
	return length;
}


void main(void)
{
	int ret;

	setup_disk();

	printk("disk setup complete!\n");
	k_sleep(K_SECONDS(2));
	printk("now attempting to create test files..\n");
	create_test_files();

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	LOG_INF("Main Mass Application\n");
	LOG_INF("The device is put in USB mass storage mode.\n");
}



