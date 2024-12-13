#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "modules/utils.c"
#include "modules/imu.h"

#include "esp_timer.h"

#include "esp_log.h"

uint16_t roll, pitch, yaw, throttle;

// IMU
bool left_calibrated = false;
bool right_calibrated = false;

void switch_init() {
    /*Switch Mode Init pin : D5 or GPIO 5 */
    gpio_config_t io_conf1 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_5),  // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf1);

    /*Switch Mode Init pin : D4 or GPIO 4 */
    gpio_config_t io_conf2 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_4),  // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf2);

    /*Switch Mode Init pin : D19 or GPIO 19 */
    gpio_config_t io_conf3 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_19), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf3);

    /*Switch Mode Init pin : D26 or GPIO 26 */
    gpio_config_t io_conf4 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_26), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf4);

    /*Switch Mode Init pin : D15 or GPIO 15 */
    gpio_config_t io_conf5 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_15), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf5);

}

// IMU related task
void i2c_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void left_imu_task() {
    struct full_imu_data imu_data = create_full_imu_data();

    // Initialize MPU6050
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &imu_data.gyro, &imu_data.acce);

        imu_add(&imu_data.offset, imu_data.gyro);
    }
    imu_divide_single(&imu_data.offset, 2000);
    printf("Left Gyro Calibration done!!\n");
    left_calibrated = true;

    while (true) {
        imu_read(imu, &imu_data);

        /*Mapping degree to PWM*/
        throttle = mapValue(imu_data.processed.x, -30, 30, 1000, 2000);
        yaw = mapValue(imu_data.processed.y, -30, 30, 1000, 2000);

        // Deadzone
        // if (yaw_pwm > 1450 && yaw_pwm < 1550) {
        //     yaw_pwm = 1500;
        // } else if (yaw_pwm > 1450) {
        //     yaw_pwm += 50;
        // } else if (yaw_pwm < 1550) {
        //     yaw_pwm -= 50;
        // }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void right_imu_task() {
    struct full_imu_data imu_data = create_full_imu_data();

    // Initialize MPU6050
    mpu6050_handle_t imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS);

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(imu, &imu_data.gyro, &imu_data.acce);

        imu_add(&imu_data.offset, imu_data.gyro);
    }
    imu_divide_single(&imu_data.offset, 2000);
    printf("Right Gyro Calibration done!!\n");
    right_calibrated = true;

    while (true) {
        imu_read(imu, &imu_data);

        /*Mapping degree to PWM*/
        roll = mapValue(imu_data.processed.y, -45, 45, 1000, 2000);
        pitch = mapValue(imu_data.processed.x, -45, 45, 2000, 1000);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ELRS related task

void elrs_send_payload_task() {
}

void send_payload_task() {
    while (true) {
        if (left_calibrated && right_calibrated) {
            printf("r%dp%dy%dt%d\n", roll, pitch, yaw, throttle);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    i2c_init();
    // switch_init();

    xTaskCreate(left_imu_task, "left_imu", 4096, NULL, 4, NULL);
    xTaskCreate(right_imu_task, "right_imu", 4096, NULL, 4, NULL);
    xTaskCreate(send_payload_task, "send_payload", 4096, NULL, 5, NULL);
}