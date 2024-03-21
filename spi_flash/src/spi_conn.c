#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>


void test_cmd(){
    struct device* spi_dev_flash = DEVICE_DT_GET(DT_NODELABEL(spi3));
    
    struct spi_config spi_cfg_flash =
{
    .frequency = 800000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    // this should work, but for some reason it doesn't. However, this might work without
    // the cs field anyway, apparently it is auto managed, so test first
    //.cs = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spi2), 0)
    };
    uint8_t op_code;

    uint8_t rx[2]; 
    const struct spi_buf tx = {
        .buf = &op_code,
        .len = 1
    };
    const struct spi_buf rx_buf = {
        .buf = rx,
        .len = 1
    };

    const struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};

    const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};
    spi_transceive(spi_dev_flash, &spi_cfg_flash, &tx_set, &rx_set);
}
