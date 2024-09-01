
/*
#ifndef __ASSEMBLER__
    #include "/home/sanch/prog/esp32/BMI160-SPI/main/debug.h"
#endif

#define traceTASK_SWITCHED_IN() TaskSwitchedIn((int)pxCurrentTCBs[ portGET_CORE_ID() ]->pxTaskTag);
#define traceTASK_SWITCHED_OUT() TaskSwitchedOut((int)pxCurrentTCBs[ portGET_CORE_ID() ]->pxTaskTag);
*/

#include "driver/gpio.h"
#include "hal/gpio_types.h"

#define GPIO_TASK1 35
#define GPIO_TASK2 36
#define GPIO_TASK3 37

void SetupGPIOTaskSwitch() {
    gpio_config_t io_conf1;
    io_conf1.intr_type = GPIO_INTR_DISABLE;
    io_conf1.mode = GPIO_MODE_OUTPUT;
    io_conf1.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf1.pull_up_en = 0;
    io_conf1.pin_bit_mask = (1ULL << GPIO_TASK1);
    gpio_config(&io_conf1);

    gpio_config_t io_conf2;
    io_conf2.intr_type = GPIO_INTR_DISABLE;
    io_conf2.mode = GPIO_MODE_OUTPUT;
    io_conf2.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf2.pull_up_en = 0;
    io_conf2.pin_bit_mask = (1ULL << GPIO_TASK2);
    gpio_config(&io_conf2);

    gpio_config_t io_conf3;
    io_conf3.intr_type = GPIO_INTR_DISABLE;
    io_conf3.mode = GPIO_MODE_OUTPUT;
    io_conf3.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf3.pull_up_en = 0;
    io_conf3.pin_bit_mask = (1ULL << GPIO_TASK3);
    gpio_config(&io_conf3);
}

void TaskSwitchedIn(int tag) {
    switch (tag) {
        case 1:
            gpio_set_level(GPIO_TASK1, 1);
            break;
        case 2:
            gpio_set_level(GPIO_TASK2, 1);
            break;
        case 3:
            gpio_set_level(GPIO_TASK3, 1);
            break;
    }
}

void TaskSwitchedOut(int tag) {
    switch (tag) {
        case 1:
            gpio_set_level(GPIO_TASK1, 0);
            break;
        case 2:
            gpio_set_level(GPIO_TASK2, 0);
            break;
        case 3:
            gpio_set_level(GPIO_TASK3, 0);
            break;
    }
}