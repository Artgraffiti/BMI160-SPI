#pragma once

// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
#include "freertos/idf_additions.h"
#include "hal/spi_types.h"
#include "soc/gpio_num.h"

#define CMD_READ 0x01
#define CMD_WRITE 0x00

struct BMI160AccelGyroData {
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
};

void bmi160_spi_init(spi_host_device_t host_id);

void bmi160_data_rdy_int_init(gpio_num_t gpio_num);

void data_ready_isr_handler(void *pvParameters);

void bmi160_read_data_task(void *pvParameters);
