zephyr_library()

zephyr_library_sources_if_def(CONFIG_DISK_DRIVER_RAW_NAND, nand_disk.c)
zephyr_library_sources(spi_nand.c)