#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <math.h>
#include <stdio.h>
#include <esp_timer.h>
#include <stdbool.h>
#include <esp_log.h>

#include "hardware.h"
#include "dynamixel.h"
#include "networking.h"
#include "five_bar.h"
#include "main.h"
#include "motor.h"
#include "uart.h"

// dynamixel_t dx;

volatile bool fan_enabled;

// int32_t left_trim = 0;
// int32_t right_trim = 0;

void app_main() {
	gpio_reset_pin(GPIO_NUM_2); //default RTS and CTS for uart0
	gpio_reset_pin(GPIO_NUM_4);


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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    led_channel_conf.gpio_num = PIN_LED1;
    led_channel_conf.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    led_channel_conf.gpio_num = PIN_LED2;
    led_channel_conf.channel = LEDC_CHANNEL_2;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    led_channel_conf.gpio_num = PIN_LED3;
    led_channel_conf.channel = LEDC_CHANNEL_3;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

    led_channel_conf.gpio_num = PIN_LED4;
    led_channel_conf.channel = LEDC_CHANNEL_4;
    ledc_channel_config(&led_channel_conf);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_FAN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_FAN, 0);

    fan_enabled = false;


    gpio_set_direction(8, GPIO_MODE_OUTPUT);
    gpio_set_level(8, 1);

    #ifndef LOGGING_ENABLED
    esp_log_level_set("*", ESP_LOG_NONE);		//DISABLE ESP32 LOGGING ON UART0
	gpio_reset_pin(GPIO_NUM_1);
	gpio_reset_pin(GPIO_NUM_3);
    xTaskCreate(uart_thread, "uart_thread", 4086, NULL, 5, NULL);
    #endif

    xTaskCreate(motor_thread, "motor_thread", 4086, NULL, 5, NULL);
    xTaskCreate(tcp_server_thread, "tcp_server_thread", 4086, NULL, 5, NULL);

    printf("Initialised\n");

    while (true) {
        bool level = gpio_get_level(PIN_BUTTON);
        static int64_t last_toggled = 0;
        if (level == 0 && esp_timer_get_time() - last_toggled > 1e6) {
            fan_enabled = !fan_enabled;
            last_toggled = esp_timer_get_time();
            gpio_set_level(PIN_FAN, fan_enabled);
        }
        vTaskDelay(10);
    }
}