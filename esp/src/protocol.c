#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <driver/gpio.h>

#include "motor.h"
#include "main.h"
#include "hardware.h"

#define ID_HANDSHAKE 2
#define ID_CONTROL 3
#define ID_READ_POS 4
#define ID_SET_FAN 5

uint8_t id_to_len(uint8_t id) {
    switch(id) {
        case ID_HANDSHAKE:
            return 0;
        case ID_CONTROL:
            return 12;
        case ID_SET_FAN:
            return 1;
    }

    return 0;
}


// returns the length of the out buf
uint8_t handle_message(uint8_t id, uint8_t *in_buf, uint8_t *out_buf) {
    switch (id) {
        case ID_HANDSHAKE:
            out_buf[0] = 1;
            return 1;
        case ID_CONTROL:
            memcpy(&wanted_pos_x, in_buf, 4);
            memcpy(&wanted_pos_y, in_buf + 4, 4);
            memcpy(&wanted_velocity, in_buf + 8, 4);
            out_buf[0] = 1;
            return 1;
        case ID_READ_POS:
            memcpy(out_buf, &current_pos_x, 4);
            memcpy(out_buf + 4, &current_pos_y, 4);
            return 8;

        case ID_SET_FAN:
            fan_enabled = in_buf[0];
            gpio_set_level(PIN_FAN, fan_enabled);
            out_buf[0] = 1;
            return 1;
    }

    return 0;
}