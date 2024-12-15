#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "modules/utils.c"
#include "modules/imu.h"
#include "modules/elrs.h"

#include "esp_timer.h"

#include "esp_log.h"



static const char* TAG = "Tes";


// ELRS UART
#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16

//TIMER
#define INTERVALMS 3 * 1000 // in microseconds
static void timer_callback(void* arg);
bool transmit_yet = 0;

// IMU
bool left_calibrated = false;
bool right_calibrated = false;

uint16_t channels[16] = {0};


void timer_init() {
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "timer_callback"
    };

    esp_timer_handle_t timer_periodic;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_periodic));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_periodic, INTERVALMS));
}

static int64_t last_time = 0;

static void timer_callback(void* arg) {
    int64_t time_since_boot = esp_timer_get_time();
    int64_t interval = time_since_boot - last_time;
    
    // portENTER_CRITICAL_ISR(&spinlock);
    transmit_yet = interval < 4000 ? 1 : 0;
    // portEXIT_CRITICAL(&spinlock);

    // transmit_yet = 1;
    if(!transmit_yet) ESP_LOGI(TAG, "interval is %lld us", interval);
    last_time = time_since_boot;
}


void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 921600, //912600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver
    uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);
    // Set pins
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set UART mode to half-duplex
    uart_set_mode(UART_NUM, UART_MODE_UART);
    // Invert RX and TX signals
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV);
    // UART_SIGNAL_RXD_INV
}

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

    /*Switch Mode Init` pin : D4 or GPIO 4 */
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
        channels[THROTTLE] = mapValue(imu_data.processed.x, -30, 30, 0, MAX_CHANNEL_VALUE);
        channels[YAW] = mapValue(imu_data.processed.y, -30, 30, 0, MAX_CHANNEL_VALUE);

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
        channels[ROLL] = mapValue(imu_data.processed.y, -45, 45, 0, MAX_CHANNEL_VALUE);
        channels[PITCH] = mapValue(imu_data.processed.x, -45, 45, MAX_CHANNEL_VALUE, 0);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


static void switch_id() {
    uint8_t packetCommand[10] = {
        0xC8,
        0x08,
        0x32,
        DEVICE_ADDRESS_TX_MODULE,
        DEVICE_ADDRESS_REMOTE_CONTROL,
        0x10,
        0x05,
        0x01,
    };

    packetCommand[8] = get_command_crc8(&packetCommand[2], 6);
    packetCommand[9] = get_crc8(&packetCommand[2], 7);

       if( transmit_yet){
        for(int i = 0; i < 10; i++){
            printf(". ");
            elrs_send_data(UART_NUM, packetCommand, 10);
            vTaskDelay(4 / configTICK_RATE_HZ);
            }
    }

}

void elrs_task(void * pvParameters) {
    uint8_t packet[PACKET_LENGTH] = {0};
    packet[0] = 0xC8;

    while (true){   
    if(transmit_yet){
        create_crsf_channels_packet(channels, packet);
        elrs_send_data(UART_NUM, packet, PACKET_LENGTH);
        transmit_yet = 0;
    }

    vTaskDelay(1 / portTICK_PERIOD_MS); // ojo diganti
    }
}

void app_main(void) {
    uart_init();
    i2c_init();
    timer_init();
    switch_id();
    // switch_init();

    // xTaskCreate(left_imu_task, "left_imu", 4096, NULL, 4, NULL);
    // xTaskCreate(right_imu_task, "right_imu", 4096, NULL, 4, NULL);
    xTaskCreate(elrs_task, "send_payload", 4096, NULL, tskIDLE_PRIORITY, NULL);
}