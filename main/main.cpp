#include "driver/spi_common.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "bmi160.h"
#include <cstdint>
#include <cstdlib>

#define IMU_HOST        SPI2_HOST
#define PIN_NUM_CS      11
#define PIN_NUM_MISO    12
#define PIN_NUM_MOSI    13
#define PIN_NUM_CLK     14

static const char *TAG = "MAIN";

extern "C" {
    void app_main(void);
}

void bmi160(void *pvParameters);

int8_t user_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);


spi_device_handle_t spi;


void spi_init() {
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", IMU_HOST + 1);
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };
    
    spi_device_interface_config_t devcfg = {
        .mode = 0,                              // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
        .spics_io_num = PIN_NUM_CS,             // CS pin
        .queue_size = 1,                        // We want to be able to queue 1 transactions at a time
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(IMU_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    // Add device to bus
    ret = spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    spi_init();

    uint8_t chip_id;
    user_spi_read(0x00, 0x00, &chip_id, 1);
    ESP_LOGI(TAG, "CHIP_ID=%02X", chip_id);

    // Create task
    xTaskCreate(bmi160, "IMU", 1024*6, NULL, 1, NULL);
}