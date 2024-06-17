#include <driver/uart.h>
#include <math.h>
#include <stdio.h>
#include <esp_timer.h>
#include <stdbool.h>

#include "motor.h"
#include "dynamixel.h"
#include "hardware.h"
#include "five_bar.h"

dynamixel_t dx;

volatile float wanted_pos_x = 0;
volatile float wanted_pos_y = 30;
volatile float wanted_velocity = 50;
volatile float current_pos_x = 0;
volatile float current_pos_y = 0;
const float time_step = 0.001;
const float overshoot = time_step * 3;

volatile int32_t left_trim = 0;
volatile int32_t right_trim = 0;

void motor_thread() {
    uart_config_t uart_servo = {
        .baud_rate = BAUD_UARD_SERVO,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(1, &uart_servo);
    uart_set_pin(NUM_UART_SERVO, PIN_SERVO_TX, PIN_SERVO_RX, -1, -1);
    uart_driver_install(NUM_UART_SERVO, 512, 512, 20, NULL, 0);
    vTaskDelay(200);

    float virtual_current_pos_x = wanted_pos_x;
    float virtual_current_pos_y = wanted_pos_y;
    float real_current_pos_x = 0;
    float real_current_pos_y = 0;
    static const int32_t center = 4095 / 2;
    int32_t real_current_discrete_left = 0;
    int32_t real_current_discrete_right = 0;
    int32_t virtual_current_discrete_left = 0;
    int32_t virtual_current_discrete_right = 0;
    int32_t last_update = esp_timer_get_time();
    dx_init(&dx, NUM_UART_SERVO);

    dx_restart(&dx, LEFT_ID);
    dx_restart(&dx, RIGHT_ID);
    vTaskDelay(1000);

    dx_set_status_return_level(&dx, DX_ID_BROADCAST, DX_PING_READ);
    dx_set_operating_mode(&dx, LEFT_ID, DX_POSITION_CONTROL);
    dx_set_operating_mode(&dx, RIGHT_ID, DX_POSITION_CONTROL);
    dx_enable_torque(&dx, LEFT_ID, true);
    dx_enable_torque(&dx, RIGHT_ID, true);

    dx_set_profile_velocity_raw(&dx, LEFT_ID, 100);
    dx_set_profile_velocity_raw(&dx, RIGHT_ID, 100);
    vTaskDelay(500);

    float l,r;

    five_bar_pos_to_angle(R1, R2, R3, R4, R5, virtual_current_pos_x, virtual_current_pos_y, &l, &r);

    float this_step_left_adjusted = -(PI / 2 - l);
    float this_step_right_adjusted = -(PI / 2 - r);

    int32_t left_discrete = center + this_step_left_adjusted / (2 * PI) * 4095 + left_trim;
    int32_t right_discrete = center + this_step_right_adjusted / (2 * PI) * 4095 + right_trim;

    dx_set_goal_position(&dx, LEFT_ID, left_discrete, false);
    dx_set_goal_position(&dx, RIGHT_ID, right_discrete, false);
    vTaskDelay(1000);
    virtual_current_discrete_left = left_discrete;
    virtual_current_discrete_right = right_discrete;
    dx_set_profile_velocity_raw(&dx, LEFT_ID, 0);
    dx_set_profile_velocity_raw(&dx, RIGHT_ID, 0);
    vTaskDelay(100);
    last_update = esp_timer_get_time();
    while (true) {
        float scale = 1;
        if (wanted_pos_x < BOARD_LEFT_LIM) {
            wanted_pos_x = BOARD_LEFT_LIM;
        }
        if (wanted_pos_x > BOARD_RIGHT_LIM) {
            wanted_pos_x = BOARD_RIGHT_LIM;
        }
        if (wanted_pos_y < BOARD_BOT_LIM) {
            wanted_pos_y = BOARD_BOT_LIM;
        }
        if (wanted_pos_y > BOARD_TOP_LIM) {
            wanted_pos_y = BOARD_TOP_LIM;
        }


        float wanted_delta_x = wanted_pos_x - virtual_current_pos_x;
        float wanted_delta_y = wanted_pos_y - virtual_current_pos_y;

        float wanted_delta_abs = sqrtf(wanted_delta_x * wanted_delta_x + wanted_delta_y * wanted_delta_y);

        float total_move_time = wanted_delta_abs / wanted_velocity;

        float step_move_time = total_move_time > time_step + overshoot ? time_step + overshoot : total_move_time;

        float this_step_delta_x = wanted_delta_x * step_move_time / total_move_time;
        float this_step_delta_y = wanted_delta_y * step_move_time / total_move_time;

        if (total_move_time == 0) {
            this_step_delta_x = 0;
            this_step_delta_y = 0;
            goto readback;
        }

        float this_step_pos_x = virtual_current_pos_x + this_step_delta_x;
        float this_step_pos_y = virtual_current_pos_y + this_step_delta_y;

        float this_step_left_angle;
        float this_step_right_angle;
        five_bar_pos_to_angle(R1, R2, R3, R4, R5, this_step_pos_x, this_step_pos_y, &this_step_left_angle, &this_step_right_angle);

        float this_step_left_adjusted = -(PI / 2 - this_step_left_angle);
        float this_step_right_adjusted = -(PI / 2 - this_step_right_angle);

        int32_t this_step_left_discrete = center + this_step_left_adjusted / (2 * PI) * 4095 + left_trim;
        int32_t this_step_right_discrete = center + this_step_right_adjusted / (2 * PI) * 4095 + right_trim;

        // speed calculations seem to be unnecessary :/

        // int32_t this_step_discrete_delta_left = this_step_left_discrete - virtual_current_discrete_left;
        // int32_t this_step_discrete_delta_right = this_step_right_discrete - virtual_current_discrete_right;
        // virtual_current_discrete_left = this_step_left_discrete;
        // virtual_current_discrete_right = this_step_right_discrete;

        // float steps_per_second_left = this_step_discrete_delta_left / step_move_time;
        // float steps_per_second_right = this_step_discrete_delta_right / step_move_time;

        // static const float steps_per_second_to_rpm = 360 * 60.0 / 4095.0;

        // int32_t left_speed_discrete = steps_per_second_left * steps_per_second_to_rpm * 0.02; // magic constant from dynamixel docs
        // int32_t right_speed_discrete = steps_per_second_right * steps_per_second_to_rpm * 0.02;
        // printf("bajs %d\n", left_speed_discrete);
        // left_speed_discrete = abs(left_speed_discrete) < 1 ? 1 : abs(left_speed_discrete);
        // right_speed_discrete = abs(right_speed_discrete) < 1 ? 1 : abs(right_speed_discrete);

        // left_speed_discrete = abs(left_speed_discrete) > 32767 ? 32767 : abs(left_speed_discrete);
        // right_speed_discrete = abs(right_speed_discrete) > 32767 ? 32767 : abs(right_speed_discrete);


        // left_speed_discrete = 0;
        // right_speed_discrete = 0;

        // dx_set_profile_velocity_raw(&dx, LEFT_ID, left_speed_discrete);
        // dx_set_profile_velocity_raw(&dx, RIGHT_ID, right_speed_discrete);

        // printf("wanted speed %ld %ld\n", left_speed_discrete, right_speed_discrete);
        // printf("wanted angul %f %f\n", this_step_left_angle, this_step_right_angle);
        // printf("wanted disco %ld %ld\n", this_step_left_discrete, this_step_right_discrete);

        dx_set_goal_position(&dx, LEFT_ID, this_step_left_discrete, false);
        dx_set_goal_position(&dx, RIGHT_ID, this_step_right_discrete, false);
        dx_action(&dx, DX_ID_BROADCAST);
        readback:
        {
            // real_current_discrete_left = virtual_current_discrete_left;
            // real_current_discrete_right = virtual_current_discrete_right;
            // // read pos
            // uint8_t ids[] = {LEFT_ID, RIGHT_ID};
            // dx_response_t resp;
            // int ret;
            // dx_read_multiple_present_positions(&dx, ids, 2);

            // uint8_t rx_buf[2048];
            // int bytes_read = uart_read_bytes(NUM_UART_SERVO, rx_buf, sizeof(rx_buf), time_step * 1000);
            // if (bytes_read == -1) {
            //     continue;
            // }
            // for (int i = 0; i < bytes_read; i++) {
            //     int got_message = dx_parse_byte(&dx, rx_buf[i], &resp);
            //     if (got_message) {
            //         int32_t angle = 0;
            //         angle += resp.params[0];
            //         angle += resp.params[1] << 8;
            //         angle += resp.params[2] << 16;
            //         angle += resp.params[3] << 24;

            //         if (resp.id == LEFT_ID) {
            //             real_current_discrete_left = angle;
            //         } else if (resp.id == RIGHT_ID) {
            //             real_current_discrete_right = angle;
            //         }
            //     }
            // }

            int32_t dt = esp_timer_get_time() - last_update;
            last_update = esp_timer_get_time();

            if (this_step_delta_x != 0 || this_step_delta_y != 0) {
                virtual_current_pos_x += wanted_delta_x * (dt / 1000000.0) / total_move_time;
                virtual_current_pos_y += wanted_delta_y * (dt / 1000000.0) / total_move_time;
            }

            // virtual_current_discrete_left = real_current_discrete_left;
            // virtual_current_discrete_right = real_current_discrete_right;

            current_pos_x = virtual_current_pos_x;
            current_pos_y = virtual_current_pos_y;
            float left_angle = (virtual_current_discrete_left - left_trim - center) / 4095.0 * 2.0 * PI;
            float right_angle = (virtual_current_discrete_right - right_trim - center) / 4095.0 * 2.0 * PI;

            float left_adjusted = left_angle + PI / 2;
            float right_adjusted = right_angle + PI / 2;

            five_bar_angle_to_pos(R1, R2, R3, R4, R5, left_adjusted, right_adjusted, &real_current_pos_x, &real_current_pos_y);
            // virtual_current_pos_x = real_current_pos_x;
            // virtual_current_pos_y = real_current_pos_y;

            // printf("wanted pos %f %f\n", wanted_pos_x, wanted_pos_y);
            // printf("virtual pos %f %f\n", virtual_current_pos_x, virtual_current_pos_y);
            // printf("dt %d\n", dt / 1000);

            // printf("-------------------\n");

            vTaskDelay(time_step * 1000);
        }
    }
}