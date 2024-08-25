// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
#include "freertos/idf_additions.h"

#define CMD_READ 0x01
#define CMD_WRITE 0x00

struct AccelGyroData {
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
};
