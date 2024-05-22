#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <math.h>
#include <stdio.h>

#include "dynamixel.h"
#include "five_bar.h"

#define PIN_LED0 2
#define PIN_LED1 4
#define PIN_LED2 12
#define PIN_LED3 13
#define PIN_LED4 14
#define PIN_BUTTON 15
#define PIN_FAN 16
#define PIN_SERVO_RX 33
#define PIN_SERVO_TX 32
#define NUM_UART_SERVO UART_NUM_1
#define BAUD_UARD_SERVO 1000000

#define PI 3.1415

#define R1 23
#define R2 23
#define R3 23
#define R4 23
#define R5 10

#define LEFT_ID 1
#define RIGHT_ID 2


dynamixel_t dx;

volatile float wanted_pos_x = 0;
volatile float wanted_pos_y = 28;
volatile float virtual_current_position_x;
volatile float virtual_current_position_y;
volatile float wanted_velocity = 1;

int32_t left_trim;
int32_t right_trim;

void troll_thread() {
    bool ping = false;

    while(true) {
        ping = !ping;
        wanted_pos_y = 23 + ping * 10;
        vTaskDelay(5000);
    }
}

void motor_thread() {
    float current_pos_x;
    float current_pos_y;
    static const int32_t center = 4095 / 2;
    int32_t current_discrete_left = center;
    int32_t current_discrete_right = center;

    dx_init(&dx, NUM_UART_SERVO);
    dx_set_status_return_level(&dx, DX_ID_BROADCAST, DX_PING_READ);
    dx_set_operating_mode(&dx, DX_ID_BROADCAST, DX_POSITION_CONTROL);
    // dx_set_goal_position(&dx, DX_ID_BROADCAST, 4096 / 2, false);
    dx_enable_torque(&dx, DX_ID_BROADCAST, true);
    vTaskDelay(5000);

    while (true) {
        float dt = 1;
        float wanted_delta_x = wanted_pos_x - current_pos_x;
        float wanted_delta_y = wanted_pos_y - current_pos_y;

        float wanted_delta_abs = sqrtf(wanted_delta_x * wanted_delta_x + wanted_delta_y * wanted_delta_y);

        float move_time = wanted_delta_abs / wanted_velocity;

        if (move_time < 1) {
            move_time = 1;
        }

        float this_step_delta_x = wanted_delta_x * (dt / move_time);
        float this_step_delta_y = wanted_delta_y * (dt / move_time);

        float this_step_pos_x = current_pos_x + this_step_delta_x;
        float this_step_pos_y = wanted_pos_y + this_step_delta_y;

        float this_step_left_angle;
        float this_step_right_angle;
        five_bar_pos_to_angle(R1, R2, R3, R4, R5, this_step_pos_x, this_step_pos_y, &this_step_left_angle, &this_step_right_angle);

        float this_step_left_adjusted = -(PI / 2- this_step_left_angle);
        float this_step_right_adjusted = -(PI / 2 - this_step_right_angle);

        int32_t this_step_left_discrete = center + this_step_left_adjusted / (2 * PI) * 4095 + left_trim;
        int32_t this_step_right_discrete = center + this_step_right_adjusted / (2 * PI) * 4095 + right_trim;

        int32_t this_step_discrete_delta_left = this_step_left_discrete - current_discrete_left;
        int32_t this_step_discrete_delta_right = this_step_right_discrete - current_discrete_right;
        float steps_per_second_left = this_step_discrete_delta_left / move_time;
        float steps_per_second_right = this_step_discrete_delta_right / move_time;

        static const float steps_per_second_to_rpm = 60.0 / 4095.0;

        int32_t left_speed_discrete = steps_per_second_left * steps_per_second_to_rpm; // magic constant from dynamixel docs
        int32_t right_speed_discrete = steps_per_second_right * steps_per_second_to_rpm;
        left_speed_discrete = left_speed_discrete == 0 ? 1 : fabsf(left_speed_discrete);
        right_speed_discrete = right_speed_discrete == 0 ? 1 : fabsf(right_speed_discrete);
        // dx_set_goal_velocity(&dx, LEFT_ID, left_speed_discrete * 0.229, true);
        // dx_set_goal_velocity(&dx, RIGHT_ID, right_speed_discrete * 0.229, true);
        dx_action(&dx, DX_ID_BROADCAST);

        dx_set_profile_velocity_raw(&dx, LEFT_ID, left_speed_discrete);
        dx_set_profile_velocity_raw(&dx, RIGHT_ID, right_speed_discrete);

        // printf("wanted sped %ld %ld\n", left_speed_discrete, right_speed_discrete);
        // printf("wanted angul %f %f\n", this_step_left_angle, this_step_right_angle);
        // printf("wanted disco %ld %ld\n", this_step_left_discrete, this_step_right_discrete);

        dx_set_goal_position(&dx, LEFT_ID, this_step_left_discrete, false);
        dx_set_goal_position(&dx, RIGHT_ID, this_step_right_discrete, false);
        dx_action(&dx, DX_ID_BROADCAST);

        // read pos
        uint8_t ids[] = {LEFT_ID, RIGHT_ID};
        dx_response_t resp;
        int ret;
        dx_read_multiple_present_positions(&dx, ids, 2);

        uint8_t rx_buf[512];
        int bytes_read = uart_read_bytes(NUM_UART_SERVO, rx_buf, sizeof(rx_buf), dt * 1000);
        if (bytes_read == -1) {
            continue;
        }
        for (int i = 0; i < bytes_read; i++) {
            int got_message = dx_parse_byte(&dx, rx_buf[i], &resp);
            if (got_message) {
                int32_t angle = 0;
                angle += resp.params[0];
                angle += resp.params[1] << 8;
                angle += resp.params[2] << 16;
                angle += resp.params[3] << 24;

                if (resp.id == LEFT_ID) {
                    current_discrete_left = angle;
                } else if (resp.id == RIGHT_ID) {
                    current_discrete_right = angle;
                }

                // printf("disco %ld %ld\n", current_discrete_left, current_discrete_right);
            }
        }

        float left_angle = (current_discrete_left - left_trim - center) / 4095.0 * 2.0 * PI;
        float right_angle = (current_discrete_right - right_trim - center) / 4095.0 * 2.0 * PI;

        float left_adjusted = left_angle + PI / 2;
        float right_adjusted = right_angle + PI / 2;

        five_bar_angle_to_pos(R1, R2, R3, R4, R5, left_adjusted, right_adjusted, &current_pos_x, &current_pos_y);
        // current_pos_x = this_step_pos_x;
        // current_pos_y = this_step_pos_y;
        // printf("angul %f %f\n", left_adjusted, right_adjusted);
        // printf("pos %f %f\n", current_pos_x, current_pos_y);
        // printf("wanted pos %f %f\n", wanted_pos_x, wanted_pos_y);
        // printf("-------------------\n");


    }
}

void app_main() {
    ledc_timer_config_t led_timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_APB_CLK};
    ledc_timer_config(&led_timer_conf);

    ledc_channel_config_t led_channel_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED0,
        .duty = 0,
        .hpoint = 0,
    };

    led_channel_conf.gpio_num = PIN_LED0;
    led_channel_conf.channel = LEDC_CHANNEL_0;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 125);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    led_channel_conf.gpio_num = PIN_LED1;
    led_channel_conf.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 125);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    led_channel_conf.gpio_num = PIN_LED2;
    led_channel_conf.channel = LEDC_CHANNEL_2;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 125);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    led_channel_conf.gpio_num = PIN_LED3;
    led_channel_conf.channel = LEDC_CHANNEL_3;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 125);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

    led_channel_conf.gpio_num = PIN_LED4;
    led_channel_conf.channel = LEDC_CHANNEL_4;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 125);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_FAN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_FAN, 1); // fan ON or OFF
    uart_config_t uart_servo = {
        .baud_rate = BAUD_UARD_SERVO,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(1, &uart_servo);
    uart_set_pin(NUM_UART_SERVO, PIN_SERVO_TX, PIN_SERVO_RX, GPIO_NUM_NC, GPIO_NUM_NC);
    uart_driver_install(NUM_UART_SERVO, 512, 512, 20, NULL, 0);


    // TESTS using motor_thread
    // xTaskCreate(motor_thread, "motor_thread", 22000, NULL, 5, NULL);
    // xTaskCreate(troll_thread, "troll_thread", 22000, NULL, 5, NULL);
    // // dx_set_current_limit(&dx, bajs, 2000.0 / 2.6); // 200mA / 2.6mA



    // TESTS not using motor_thread()
    dx_init(&dx, NUM_UART_SERVO); // OBS this line is already in motor_thread, needed to initialize uart
    vTaskDelay(500);

    vTaskDelay(50);
    printf("1\n");
    dx_set_operating_mode(&dx, 1, DX_POSITION_CONTROL);
    vTaskDelay(50);
    printf("2\n");
    dx_enable_torque(&dx, 1, true);
    vTaskDelay(50);
    printf("3\n");
    dx_set_profile_velocity(&dx, 1, 5);
    vTaskDelay(50);
    printf("4\n");
    dx_set_goal_position(&dx, 1, 4096 / 2, false);
    vTaskDelay(100);

    vTaskDelay(10);
    printf("5\n");
    dx_set_operating_mode(&dx, 2, DX_POSITION_CONTROL);
    vTaskDelay(10);
    printf("6\n");
    dx_enable_torque(&dx, 2, true);
    vTaskDelay(10);
    printf("7\n");
    dx_set_profile_velocity(&dx, 2, 5);
    vTaskDelay(10);
    printf("8\n");
    dx_set_goal_position(&dx, 2, 4096 / 2, false);
    vTaskDelay(5000);



    // TEST untitled
    // dx.expected_self_bytes = 0;
    // uart_flush(NUM_UART_SERVO);
    // while (true) {
    //     printf("big chungus\n");
    //     dx_set_goal_position(&dx, bajs, 4096 / 2, false);
    //     uint8_t ids[] = {1, 2};
    //     dx_read_multiple_present_positions(&dx, ids, 2);
    //     uint8_t rx_buf[512];
    //     int ret;
    //     ret = uart_read_bytes(NUM_UART_SERVO, rx_buf, sizeof(rx_buf), 1000);
    //     if (ret == -1) {
    //         continue;
    //     }
    //     for (int i = 0; i < ret; i++) {
    //         dx_parse_byte(&dx, rx_buf[i]);
    //     }
    // }

    // vTaskDelay(1);
    // dx_set_goal_velocity(&dx, 1, 0, false);
    // vTaskDelay(1);

    printf("Initialised");
}