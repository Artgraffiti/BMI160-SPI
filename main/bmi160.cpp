#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "driver/spi_master.h"

// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
#include <cstdint>

// #define __DEBUG__

#define CMD_READ    0x80
#define CMD_WRITE   0x7F

static const char *TAG = "IMU";

extern spi_device_handle_t spi;

/* IMU Data */
struct bmi160_dev sensor;

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;


int8_t user_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Обнуляем структуру транзакции

    // Выделение памяти с выравниванием по 32 бита для большей эффективности транзакций
    uint8_t *send_data = (uint8_t*) heap_caps_aligned_alloc(32, 1, MALLOC_CAP_DMA);
    uint8_t *recv_data = (uint8_t*) heap_caps_aligned_alloc(32, len+1, MALLOC_CAP_DMA);

    if (send_data == NULL || recv_data == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буферов");
        if (send_data) free(send_data);
        if (recv_data) free(recv_data);
        return -1;  // Возвращаем ошибку
    }
    
    send_data[0] = reg_addr | CMD_READ;  // Подготовка адреса регистра с командой чтения
    t.length = 8 * (len + 1);  // Длина передачи в битах (адрес регистра + длина данных)
    t.tx_buffer = send_data;   // Указатель на буфер передачи
    t.rx_buffer = recv_data;   // Указатель на буфер приема
    t.user = (void*)dev_addr;  // Пользовательские данные (адрес устройства)

    ret = spi_device_polling_transmit(spi, &t);  // Выполняем транзакцию
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при чтении данных по SPI: %s", esp_err_to_name(ret));
        heap_caps_free(send_data);
        heap_caps_free(recv_data);
        return -1;  // Возвращаем ошибку
    }

    // Копируем полученные данные в буфер read_data (исключая первый байт)
    memcpy(read_data, &recv_data[1], len);

    // Освобождаем выделенную память
    heap_caps_free(send_data);
    heap_caps_free(recv_data);

#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%X, read_data=0x%X, len=%d", reg_addr, *read_data, len);
#endif
    return 0;  // Возвращаем успешное выполнение
}

int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Обнуляем структуру транзакции

    // Выделение памяти с выравниванием по 32 бита для большей эффективности транзакций
    uint8_t *send_data = (uint8_t*) heap_caps_aligned_alloc(32, len + 1, MALLOC_CAP_DMA);

    if (send_data == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буфера");
        return -1;  // Возвращаем ошибку
    }

    send_data[0] = reg_addr & CMD_WRITE;  // Первый байт - адрес регистра
    memcpy(&send_data[1], write_data, len);  // Копируем данные для записи после адреса регистра
    t.length = (len + 1) * 8;  // Длина передачи в битах
    t.tx_buffer = send_data;   // Указатель на буфер передачи
    t.user = (void*)send_data;  // Пользовательские данные (адрес устройства)

    ret = spi_device_polling_transmit(spi, &t);  // Выполняем транзакцию
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при передаче данных по SPI: %s", esp_err_to_name(ret));
        heap_caps_free(send_data);
        return -1;  // Возвращаем ошибку
    }

    // Освобождаем выделенную память
    heap_caps_free(send_data);
#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%X, write_data=0x%X, len=%d", reg_addr, *write_data, len);
#endif
    return 0;  // Возвращаем успешное выполнение
}

void user_delay_ms(uint32_t period) {
	esp_rom_delay_us(period*1000);
};


void bmi160(void *pvParameters) {
    sensor.id = 1;
    sensor.intf = BMI160_SPI_INTF;
    sensor.read = user_spi_read;
    sensor.write = user_spi_write;
    sensor.delay_ms = user_delay_ms;
	int8_t ret = bmi160_init(&sensor);
    if (ret == BMI160_OK)
	{
		ESP_LOGI(TAG, "BMI160 initialization success !");
		ESP_LOGI(TAG, "Chip ID 0x%X", sensor.chip_id);
	}
	else {
		ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
		vTaskDelete(NULL);
	}

    // Config Accel
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	accel_sensitivity = 16384.0; // g

    // Config Gyro
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	//sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	gyro_sensitivity = 131.2; // Deg/Sec

    ret = bmi160_set_sens_conf(&sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "bmi160_set_sens_conf");

    double ax, ay, az;
	double gx, gy, gz;
    struct bmi160_sensor_data accel;
	struct bmi160_sensor_data gyro;

    for(;;) {
        int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
        if (ret != BMI160_OK) {
            ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
		    vTaskDelete(NULL);
        }

	    // Convert relative to absolute
        ax = (double)accel.x / accel_sensitivity;
        ay = (double)accel.y / accel_sensitivity;
        az = (double)accel.z / accel_sensitivity;
        gx = (double)gyro.x / gyro_sensitivity;
        gy = (double)gyro.y / gyro_sensitivity;
        gz = (double)gyro.z / gyro_sensitivity;

        ESP_LOGI(TAG, "measurements:");
        ESP_LOGI(TAG, "accel: ax=%f, ay=%f, az=%f", ax, ay, az);
        ESP_LOGI(TAG, "gyro: gx=%f, gy=%f, gz=%f", gx, gy, gz);

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Never reach here
	vTaskDelete( NULL );
}
