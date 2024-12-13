#include "driver/gpio.h"

typedef enum {
    DEVICE_ADDRESS_BROADCAST = 0x00,
    DEVICE_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    DEVICE_ADDRESS_REMOTE_CONTROL = 0xEA,
    DEVICE_ADDRESS_RC_TRANSMITTER = 0xEE,
} crsf_device_address_t;

typedef enum {
    FRAME_TYPE_RC_CHANNELS = 0x16,
    FRAME_TYPE_PING = 0x28,
    FRAME_TYPE_DEVICE_INFO = 0x29,
    FRAME_TYPE_DIRECT_COMMANDS = 0x32,
} crsf_frame_type_t;

typedef enum {
    COMMAND_CROSSFIRE = 0x32,
    COMMAND_REMOTE_RELATED = 0x3A,
} crsf_commands_t;

typedef enum {
    CROSSFIRE_COMMAND_RECEIVER_BIND = 0x01,
    CROSSFIRE_COMMAND_CANCEL_BIND = 0x02,
    CROSSFIRE_COMMAND_SET_BIND_ID = 0x03,
    CROSSFIRE_COMMAND_MODEL_SELECT = 0x05,
    COMMAND_CROSSFIRE = 0x32,
} crsf_crossfire_commands_t;

typedef enum {
    REMOTE_RELATED_COMMANDS_TIMING_CORRECTION = 0x10,
} crsf_remote_related_commands_t;

typedef enum {
    STANDARD,
    COMMAND,
} crsf_frame_crc_type;

void prepare_crsf_frame();
void elrs_send_data(const int port, const uint8_t *data, size_t len);