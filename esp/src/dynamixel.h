#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <driver/uart.h>
#include <string.h>

#define DX_ID_BROADCAST 254

enum dx_state {
    DX_STATE_HEADER_0,
    DX_STATE_HEADER_1,
    DX_STATE_HEADER_2,
    DX_STATE_HEADER_3,
    DX_STATE_ID,
    DX_STATE_LENGTH_0,
    DX_STATE_LENGTH_1,
    DX_STATE_INSTRUCTION,
    DX_STATE_ERROR,
    DX_STATE_PARAM,
    DX_STATE_CRC_0,
    DX_STATE_CRC_1,
};

typedef enum dx_baud_rate {
    DX_BAUD_9600 = 0,
    DX_BAUD_57600 = 1,
    DX_BAUD_115200 = 2,
    DX_BAUD_1000000 = 3,
    DX_BAUD_2000000 = 4,
    DX_BAUD_3000000 = 5,
    DX_BAUD_4000000 = 6
} dx_baud_t;

typedef enum dx_operating_mode {
    DX_CURRENT_CONTROL = 0,
    DX_VELOCITY_CONTROL = 1,
    DX_POSITION_CONTROL = 3,
    DX_EXT_POSITION_CONTROL = 4,
    DX_CURRENT_POSITION_CONTROL = 5,
    DX_PWM_CONTROL = 16
} dx_operating_mode_t;

typedef enum dx_status_return_level {
    DX_PING = 0,
    DX_PING_READ = 1,
    DX_ALL = 2
} dx_status_return_level_t;

typedef struct dynamixel {
    uint8_t uart_num;
    uint16_t expected_self_bytes;
    enum dx_state state;
    uint16_t rx_params_length;
    uint16_t rx_crc;
    uint8_t rx_id;
    uint8_t rx_instruction;
    uint8_t rx_error;
    uint8_t rx_buf[128];
    uint8_t rx_buf_index;
    uint8_t got_response;
    uint16_t calculated_crc;
} dynamixel_t;

typedef struct dx_response {
    uint8_t error;
    uint8_t id;
    uint16_t length;
    uint8_t params[256];
} dx_response_t;

void dx_test(dynamixel_t *dx);
void dx_init(dynamixel_t *dx, uint8_t uart_num);
int dx_parse_byte(dynamixel_t *dx, uint8_t byte, dx_response_t *out);
void dx_ping(dynamixel_t *dx, uint8_t id);
void dx_enable_torque(dynamixel_t *dx, uint8_t id, uint8_t enable);
void dx_set_operating_mode(dynamixel_t *dx, uint8_t id, dx_operating_mode_t mode);
void dx_set_status_return_level(dynamixel_t *dx, uint8_t id, dx_status_return_level_t mode);
void dx_set_current_limit(dynamixel_t *dx, uint8_t id, uint16_t limit);

void dx_set_goal_velocity(dynamixel_t *dx, uint8_t id, float rpm, uint8_t sync);
void dx_set_goal_percent(dynamixel_t *dx, uint8_t id, float percent, uint8_t sync);
void dx_set_goal_position(dynamixel_t *dx, uint8_t id, uint16_t position, uint8_t sync);
void dx_action(dynamixel_t *dx, uint8_t id);
void dx_set_baud(dynamixel_t *dx, uint8_t id, dx_baud_t baud);
void dx_got_response(dynamixel_t *dx);
void dx_set_profile_velocity(dynamixel_t *dx, uint8_t id, float rpm);
void dx_read_multiple_present_positions(dynamixel_t *dx, uint8_t ids[], uint8_t ids_len);
void dx_set_profile_velocity_raw(dynamixel_t *dx, uint8_t id, int32_t raw);

#ifdef __cplusplus
}
#endif