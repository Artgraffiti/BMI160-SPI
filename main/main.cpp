#include <cstdlib>

#include "bmi160.hpp"
#include "imu.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

#define IMU_HOST SPI2_HOST
#define TASK_STATS_BUFFER_SIZE 1024

static const char *TAG = "MAIN";

extern "C" {
    void app_main(void);
}

extern TaskHandle_t read_data_task_handle;
extern QueueHandle_t bmi160_queue;
extern spi_device_handle_t spi;

void spi_init() {
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", IMU_HOST + 1);
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = CONFIG_GPIO_MOSI;
    buscfg.miso_io_num = CONFIG_GPIO_MISO;
    buscfg.sclk_io_num = CONFIG_GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.command_bits = 1;                   // R/W
    devcfg.address_bits = 7;                   // Register address
    devcfg.mode = 0;                           // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
    devcfg.clock_speed_hz = 10 * 1000 * 1000;  // 10 MHz
    devcfg.spics_io_num = CONFIG_GPIO_CS;      // CS pin
    devcfg.queue_size = 7;

    // Initialize the SPI bus
    ret = spi_bus_initialize(IMU_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Add device to bus
    ret = spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void stats(void *pvParameters) {
    char taskStatsBuffer[TASK_STATS_BUFFER_SIZE];

    for (;;) {
        vTaskGetRunTimeStats(taskStatsBuffer);
        printf("\nTask Runtime Stats:\nTask\t\tRun Time\tPercentage\n%s", taskStatsBuffer);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
	// Initialize SPI
    spi_init();

    // Config interrupts from bmi160
    bmi160_data_rdy_int_init();

    bmi160_queue = xQueueCreate(10, sizeof(AccelGyroData));
    if (bmi160_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        vTaskDelete(NULL);
    }

    // Create task
    xTaskCreate(bmi160, "BMI160", 1024 * 4, NULL, 5, &read_data_task_handle);
    xTaskCreate(stats, "stats", 1024 * 4, NULL, 4, NULL);

    return;
}