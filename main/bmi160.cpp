#include "esp_err.h"
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

    uint8_t *tx_buffer = (uint8_t *)calloc(1, sizeof(uint8_t));
    uint8_t *rx_buffer = (uint8_t *)calloc(len + 1, sizeof(uint8_t));

    if (tx_buffer == NULL || rx_buffer == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буферов");
        if (tx_buffer) free(tx_buffer);
        if (rx_buffer) free(rx_buffer);
        return -1;  // Возвращаем ошибку
    }
    
    tx_buffer[0] = reg_addr | CMD_READ;  // Подготовка адреса регистра с командой чтения
    t.length = 8 * (len + 1);  // Длина передачи в битах (адрес регистра + длина данных)
    t.tx_buffer = tx_buffer;   // Указатель на буфер передачи
    t.rx_buffer = rx_buffer;   // Указатель на буфер приема
    t.user = (void*)dev_addr;  // Пользовательские данные (адрес устройства)

    ret = spi_device_polling_transmit(spi, &t);  // Выполняем транзакцию
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при чтении данных по SPI: %s", esp_err_to_name(ret));
        free(tx_buffer);
        free(rx_buffer);
        return -1;  // Возвращаем ошибку
    }

    // Копируем полученные данные в буфер read_data (исключая первый байт)
    memcpy(read_data, &rx_buffer[1], len);

    // Освобождаем выделенную память
    free(tx_buffer);
    free(rx_buffer);

#ifdef __DEBUG__
    ESP_LOGI(TAG, "reg_addr=0x%X, read_data=0x%X, len=%d", reg_addr, *read_data, len);
#endif
    return 0;  // Возвращаем успешное выполнение
}

int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Обнуляем структуру транзакции

    uint8_t *tx_buffer = (uint8_t*) calloc(len + 1, sizeof(uint8_t));  // Буфер для передачи данных, включая адрес регистра

    if (tx_buffer == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буфера");
        return -1;  // Возвращаем ошибку
    }

    tx_buffer[0] = reg_addr & CMD_WRITE;  // Первый байт - адрес регистра
    memcpy(&tx_buffer[1], write_data, len);  // Копируем данные для записи после адреса регистра
    t.length = (len + 1) * 8;  // Длина передачи в битах
    t.tx_buffer = tx_buffer;   // Указатель на буфер передачи
    t.user = (void*)dev_addr;  // Пользовательские данные (адрес устройства)

    ret = spi_device_polling_transmit(spi, &t);  // Выполняем транзакцию
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при передаче данных по SPI: %s", esp_err_to_name(ret));
        free(tx_buffer);
        return -1;  // Возвращаем ошибку
    }

    // Освобождаем выделенную память
    free(tx_buffer);
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
