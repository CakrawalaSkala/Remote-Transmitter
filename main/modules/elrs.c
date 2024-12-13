#include "elrs.h"
#include <stdint.h>

#define TAG "ELRS"

#define GPIO_PIN_RCSIGNAL_TX 21
#define GPIO_PIN_RCSIGNAL_RX 21

#define MATRIX_DETACH_IN_HIGH = 0x38

#define CRSF_CRC_POLY 0xd5
#define CRSF_CRC_COMMAND_POLY 0xBA

static uint8_t get_crc8(uint8_t *buf, size_t size) {
    uint8_t crc8 = 0x00;

    for (int i = 0; i < size; i++) {
        crc8 ^= buf[i];

        for (int j = 0; j < 8; j++) {
            if (crc8 & 0x80) {
                crc8 <<= 1;
                crc8 ^= CRSF_CRC_POLY;
            } else {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}

static uint8_t get_command_crc8(uint8_t *buf, size_t size) {
    uint8_t crc8 = 0x00;

    for (int i = 0; i < size; i++) {
        crc8 ^= buf[i];

        for (int j = 0; j < 8; j++) {
            if (crc8 & 0x80) {
                crc8 <<= 1;
                crc8 ^= CRSF_CRC_COMMAND_POLY;
            } else {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}

void elrs_send_data(const int port, const uint8_t *data, size_t len) {
    if (uart_write_bytes(port, data, len) != len) {
        ESP_LOGE(TAG, "Send data critical failure.");
    }
}