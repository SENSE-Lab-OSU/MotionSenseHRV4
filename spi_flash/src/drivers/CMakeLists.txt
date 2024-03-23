zephyr_library()

zephyr_library_sources_if_def(CONFIG_CUSTOM_NAND, custom_nor_driver.c)
