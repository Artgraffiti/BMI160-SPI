#include "driver/spi_common.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include <cstdlib>

#define IMU_HOST        SPI2_HOST


static const char *TAG = "MAIN";

extern "C" {
    void app_main(void);
}

void bmi160(void *pvParameters);


spi_device_handle_t spi;


void spi_init() {
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", IMU_HOST + 1);
    spi_bus_config_t buscfg = {
        .mosi_io_num = CONFIG_GPIO_MOSI,
        .miso_io_num = CONFIG_GPIO_MISO,
        .sclk_io_num = CONFIG_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };
    
    spi_device_interface_config_t devcfg = {
        .command_bits = 1,                      // R/W
        .address_bits = 7,                      // Register address
        .mode = 0,                              // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
        .spics_io_num = CONFIG_GPIO_CS,         // CS pin
        .queue_size = 1,
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(IMU_HOST, &buscfg, 3);
    ESP_ERROR_CHECK(ret);

    // Add device to bus
    ret = spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    spi_init();

    // Create task
    xTaskCreate(bmi160, "IMU", 1024*6, NULL, 1, NULL);
}