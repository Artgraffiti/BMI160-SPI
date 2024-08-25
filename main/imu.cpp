#include "bmi160.hpp"
#include "esp_log.h"
#include "freertos/idf_additions.h"

static const char *TAG = "IMU";

extern QueueHandle_t bmiQueue;

void imu(void *pvParameters) {
    AccelGyroData data;
    double ax, ay, az;
    double gx, gy, gz;

    float accel_sensitivity = 16384.0;  // g
    float gyro_sensitivity = 131.2;     // Deg/Sec

    for (;;) {
        // ESP_LOGI(TAG, "messages in Queue: %d", uxQueueMessagesWaiting(bmiQueue));

        if (xQueueReceive(bmiQueue, &data, portMAX_DELAY)) {
            // Convert relative to absolute
            ax = (double)data.accel.x / accel_sensitivity;
            ay = (double)data.accel.y / accel_sensitivity;
            az = (double)data.accel.z / accel_sensitivity;
            gx = (double)data.gyro.x / gyro_sensitivity;
            gy = (double)data.gyro.y / gyro_sensitivity;
            gz = (double)data.gyro.z / gyro_sensitivity;
        } else {
            ESP_LOGE(TAG, "xQueueReceive fail");
            break;
        }

        // ESP_LOGI(TAG, "measurements:");
        // ESP_LOGI(TAG, "accel: ax=%f, ay=%f, az=%f", ax, ay, az);
        // ESP_LOGI(TAG, "gyro: gx=%f, gy=%f, gz=%f", gx, gy, gz);

        taskYIELD();

        // vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Never reach here
    vTaskDelete(NULL);
}
