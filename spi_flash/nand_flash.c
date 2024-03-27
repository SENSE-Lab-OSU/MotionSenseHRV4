#include "spi_nand.h"
#include <zephyr/drivers/flash.h>


static const struct flash_driver_api spi_nor_api = {
	.read = spi_nand_page_read,
	.write = spi_nand_page_read,
	.erase = ,
	.get_parameters = flash_nor_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = spi_nor_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = spi_nor_sfdp_read,
	.read_jedec_id = spi_nor_read_jedec_id,
#endif
};

#ifndef CONFIG_SPI_NOR_SFDP_RUNTIME
/* We need to know the size and ID of the configuration data we're
 * using so we can disable the device we see at runtime if it isn't
 * compatible with what we're taking from devicetree or minimal.
 */
BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, jedec_id),
	     "jedec,spi-nor jedec-id required for non-runtime SFDP");

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

/* For devicetree or minimal page layout we need to know the size of
 * the device.  We can't extract it from the raw BFP data, so require
 * it to be present in devicetree.
 */
BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size),
	     "jedec,spi-nor size required for non-runtime SFDP page layout");

/* instance 0 size in bytes */
#define INST_0_BYTES (DT_INST_PROP(0, size) / 8)

BUILD_ASSERT(SPI_NOR_IS_SECTOR_ALIGNED(CONFIG_SPI_NOR_FLASH_LAYOUT_PAGE_SIZE),
	     "SPI_NOR_FLASH_LAYOUT_PAGE_SIZE must be multiple of 4096");

/* instance 0 page count */
#define LAYOUT_PAGES_COUNT (INST_0_BYTES / CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE)

BUILD_ASSERT((CONFIG_SPI_NOR_FLASH_LAYOUT_PAGE_SIZE * LAYOUT_PAGES_COUNT)
	     == INST_0_BYTES,
	     "SPI_NOR_FLASH_LAYOUT_PAGE_SIZE incompatible with flash size");

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#ifdef CONFIG_SPI_NOR_SFDP_DEVICETREE
BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, sfdp_bfp),
	     "jedec,spi-nor sfdp-bfp required for devicetree SFDP");

static const __aligned(4) uint8_t bfp_data_0[] = DT_INST_PROP(0, sfdp_bfp);
#endif /* CONFIG_SPI_NOR_SFDP_DEVICETREE */

#endif /* CONFIG_SPI_NOR_SFDP_RUNTIME */

#if DT_INST_NODE_HAS_PROP(0, has_lock)
/* Currently we only know of devices where the BP bits are present in
 * the first byte of the status register.  Complain if that changes.
 */
BUILD_ASSERT(DT_INST_PROP(0, has_lock) == (DT_INST_PROP(0, has_lock) & 0xFF),
	     "Need support for lock clear beyond SR1");
#endif

static const struct spi_nor_config spi_nor_config_0 = {
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8),
				    CONFIG_SPI_NOR_CS_WAIT_DELAY),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif

#if !defined(CONFIG_SPI_NOR_SFDP_RUNTIME)

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.layout = {
		.pages_count = LAYOUT_PAGES_COUNT,
		.pages_size = CONFIG_SPI_NOR_FLASH_LAYOUT_PAGE_SIZE,
	},
#undef LAYOUT_PAGES_COUNT
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

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
#endif /* CONFIG_SPI_NOR_SFDP_DEVICETREE */

#endif /* CONFIG_SPI_NOR_SFDP_RUNTIME */
};

static struct spi_nor_data spi_nor_data_0;

PM_DEVICE_DT_INST_DEFINE(0, spi_nor_pm_control);
DEVICE_DT_INST_DEFINE(0, &spi_nor_init, PM_DEVICE_DT_INST_GET(0),
		 &spi_nor_data_0, &spi_nor_config_0,
		 POST_KERNEL, CONFIG_SPI_NOR_INIT_PRIORITY,
		 &spi_nor_api);
