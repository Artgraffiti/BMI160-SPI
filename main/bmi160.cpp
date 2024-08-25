#include "bmi160.hpp"

#include <cstdint>

#include "bmi160_defs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "sdkconfig.h"

// #define __DEBUG__

static const char *TAG = "BMI";

extern spi_device_handle_t spi;

/* IMU Data */
struct bmi160_dev sensor;

TaskHandle_t read_data_task_handle = NULL;

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

    // Setting up transaction
    t.length = 8 * len;
    t.cmd = CMD_READ;
    t.addr = reg_addr;
    t.rx_buffer = read_data;

    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error when reading SPI data: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%02X, len=%d", reg_addr & 0x7F, len);
    print_byte_array("read_data", read_data, len);
#endif
    return ESP_OK;
}

int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Setting up transaction
    t.length = 8 * len;
    t.cmd = CMD_WRITE;
    t.addr = reg_addr;
    t.tx_buffer = write_data;

    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error when reading SPI data: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%02X, len=%d", reg_addr & 0x7F, len);
    print_byte_array("write_data", write_data, len);
#endif
    return ESP_OK;
}

void user_delay_ms(uint32_t period) { esp_rom_delay_us(period * 1000); };

static void IRAM_ATTR data_ready_isr_handler(void *pvParameters) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(read_data_task_handle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;  // -2 --> +2[g]
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Config Gyro
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;  // -250 --> +250[Deg/Sec]
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    ret = bmi160_set_sens_conf(&sensor);
    if (ret != BMI160_OK) {
        ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "bmi160_set_sens_conf");

    // Config Interrupt on esp32
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_GPIO_DATA_RDY_INT);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CONFIG_GPIO_DATA_RDY_INT, data_ready_isr_handler, NULL);

    // Config Interrupt
    struct bmi160_int_settg int_config;
    int_config.int_channel = BMI160_INT_CHANNEL_1;
    int_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT;
    int_config.int_pin_settg.output_en = 1;                      // Output enable
    int_config.int_pin_settg.output_mode = 0;                    // push-pull mode
    int_config.int_pin_settg.output_type = 0;                    // active low
    int_config.int_pin_settg.edge_ctrl = 1;                      // edge trigger
    int_config.int_pin_settg.input_en = 0;                       // input disabled
    int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;  // non-latched output
    ret = bmi160_set_int_config(&int_config, &sensor);
    if (ret != BMI160_OK) {
        ESP_LOGE(TAG, "BMI160 set_int_conf fail %d", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "bmi160_set_int_conf");

    struct AccelGyroData data;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int8_t ret =
            bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &data.accel, &data.gyro, &sensor);
        if (ret != BMI160_OK) {
            ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
            vTaskDelete(NULL);
        }

        // if (xQueueSend(bmiQueue, &data, portMAX_DELAY) != pdPASS) {
        //     ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
        // }

#ifdef __DEBUG__
        ESP_LOGI(TAG, "RAW DATA:");
        ESP_LOGI(TAG, "ACCEL: x=%f, y=%f, z=%f", (double)data.accel.x, (double)data.accel.y,
                 (double)data.accel.z);
        ESP_LOGI(TAG, "GYRO: x=%f, y=%f, z=%f", (double)data.gyro.x, (double)data.gyro.y,
                 (double)data.gyro.z);
#endif
    }

    // Never reach here
    vTaskDelete(NULL);
}
