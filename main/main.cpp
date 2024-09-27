#include <cstdlib>

#include "bmi160.hpp"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "imu.hpp"
#include "sdkconfig.h"
#include "soc/gpio_num.h"

#define IMU_HOST SPI2_HOST
#define TASK_STATS_BUFFER_SIZE 1024

static const char *TAG = "MAIN";

extern "C" {
void app_main(void);
}

extern TaskHandle_t read_data_task_handle;
extern QueueHandle_t bmi160_queue;
extern spi_device_handle_t spi;

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
    bmi160_spi_init(IMU_HOST);

    // Config interrupts from bmi160
    bmi160_data_rdy_int_init((gpio_num_t)CONFIG_GPIO_DATA_RDY_INT);

    bmi160_queue = xQueueCreate(10, sizeof(BMI160AccelGyroData));
    if (bmi160_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        vTaskDelete(NULL);
    }

    // Create task
    xTaskCreate(bmi160_read_data_task, "BMI160", 1024 * 4, NULL, 5, &read_data_task_handle);
    xTaskCreate(stats, "stats", 1024 * 4, NULL, 4, NULL);

    return;
}