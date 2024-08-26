#include <cstdlib>

#include "bmi160.hpp"
#include "driver/gpio.h"
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

void bmi160(void *pvParameters);
void imu(void *pvParameters);

TaskHandle_t read_data_task_handle;
QueueHandle_t bmi160_queue;
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
        .command_bits = 1,                   // R/W
        .address_bits = 7,                   // Register address
        .mode = 0,                           // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        .clock_speed_hz = 10 * 1000 * 1000,  // 10 MHz
        .spics_io_num = CONFIG_GPIO_CS,      // CS pin
        .queue_size = 7,
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(IMU_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Add device to bus
    ret = spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void bmi160_data_rdy_int_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_GPIO_DATA_RDY_INT);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CONFIG_GPIO_DATA_RDY_INT, data_ready_isr_handler, NULL);
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
    xTaskCreate(bmi160, "BMI160", 1024 * 4, NULL, 2, &read_data_task_handle);
    xTaskCreate(stats, "stats", 1024 * 4, NULL, 10, NULL);

    return;
}