#include "bmi160_defs.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstdint>
#include "freertos/idf_additions.h"
#include "driver/spi_master.h"
#include "bmi160.hpp"

// #define __DEBUG__

static const char *TAG = "BMI";

extern spi_device_handle_t spi;

/* IMU Data */
struct bmi160_dev sensor;

extern QueueHandle_t bmiQueue;


#ifdef __DEBUG__
void print_byte_array(const char *label, uint8_t *array, size_t length) {
    printf("%s: ", label);
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", array[i]);
    }
    printf("\n");
}
#endif


int8_t user_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Memory allocation with 32-bit alignment for more efficient transactions with DMA
    uint8_t *recv_data = (uint8_t*) heap_caps_aligned_alloc(32, len, MALLOC_CAP_DMA);

    if (recv_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        return -1;
    }

    // Setting up transaction
    t.length = 8 * len;
    t.cmd = CMD_READ;
    t.addr = reg_addr;
    t.rx_buffer = recv_data;

    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error when reading SPI data: %s", esp_err_to_name(ret));
        heap_caps_free(recv_data);
        return -1;
    }

    // Copy the received data
    memcpy(read_data, recv_data, len);

    // Freeing allocated memory
    heap_caps_free(recv_data);

#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%02X, len=%d", reg_addr & 0x7F, len);
    print_byte_array("read_data", read_data, len);
#endif
    return 0;
}

int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Memory allocation with 32-bit alignment for more efficient transactions with DMA
    uint8_t *send_data = (uint8_t*) heap_caps_aligned_alloc(32, len, MALLOC_CAP_DMA);

    if (send_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for the buffer");
        return -1;
    }

    // Сopy information for sending
    memcpy(send_data, write_data, len);

    // Setting up transaction
    t.length = 8 * len;
    t.cmd = CMD_WRITE;
    t.addr = reg_addr;
    t.tx_buffer = send_data;

    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error when reading SPI data: %s", esp_err_to_name(ret));
        heap_caps_free(send_data);
        return -1;
    }

    // Freeing allocated memory
    heap_caps_free(send_data);

#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%02X, len=%d", reg_addr & 0x7F, len);
    print_byte_array("write_data", write_data, len);
#endif
    return 0;
}

void user_delay_ms(uint32_t period) {
	esp_rom_delay_us(period*1000);
};


void bmi160(void *pvParameters) {
    sensor.id = CONFIG_GPIO_CS;
    sensor.intf = BMI160_SPI_INTF;
    sensor.read = user_spi_read;
    sensor.write = user_spi_write;
    sensor.delay_ms = user_delay_ms;
	int8_t ret = bmi160_init(&sensor);
    if (ret == BMI160_OK) {
		ESP_LOGI(TAG, "BMI160 initialization success !");
		ESP_LOGI(TAG, "Chip ID 0x%X", sensor.chip_id);
	} else {
		ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
		vTaskDelete(NULL);
	}

    // Config Accel
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Config Gyro
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    ret = bmi160_set_sens_conf(&sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "bmi160_set_sens_conf");

    struct AccelGyroData data;

    for(;;) {
        int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &data.accel, &data.gyro, &sensor);
        if (ret != BMI160_OK) {
            ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
		    vTaskDelete(NULL);
        }

        if (xQueueSend(bmiQueue, &data, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
        }

#ifdef __DEBUG__
        ESP_LOGI(TAG, "RAW DATA:");
        ESP_LOGI(TAG, "ACCEL: x=%f, y=%f, z=%f", (double)accel.x, (double)accel.y, (double)accel.z);
        ESP_LOGI(TAG, "GYRO: x=%f, y=%f, z=%f", (double)gyro.x, (double)gyro.y, (double)gyro.z); 
#endif
        
        taskYIELD();

        // vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Never reach here
	vTaskDelete( NULL );
}
