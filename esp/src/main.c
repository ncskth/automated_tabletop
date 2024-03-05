#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include <stdio.h>

#include "dynamixel.h"

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

void app_main() {
  ledc_timer_config_t led_timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = LEDC_TIMER_8_BIT,
      .freq_hz = 1000,
      .clk_cfg = LEDC_APB_CLK
  };
  ledc_timer_config(&led_timer_conf);

  ledc_channel_config_t led_channel_conf= {
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








  // gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
  // gpio_set_direction(PIN_FAN, GPIO_MODE_OUTPUT);
  // gpio_set_level(PIN_FAN, 1);
  // uart_config_t uart_servo = {
  //   .baud_rate = BAUD_UARD_SERVO,
  //   .data_bits = UART_DATA_8_BITS,
  //   .parity = UART_PARITY_DISABLE,
  //   .stop_bits = UART_STOP_BITS_1,
  //   .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  // };

  // uart_param_config(1, &uart_servo);
  // uart_set_pin(NUM_UART_SERVO, PIN_SERVO_TX, PIN_SERVO_RX, GPIO_NUM_NC, GPIO_NUM_NC);
  // uart_driver_install(NUM_UART_SERVO, 256, 256, 20, NULL, 0);

  // dynamixel_t dx;

  // dx_init(&dx, NUM_UART_SERVO);

  // uint8_t bajs = DX_ID_BROADCAST;

  // dx_set_status_return_level(&dx, bajs, DX_PING);
  // vTaskDelay(50);
  // dx_set_current_limit(&dx, bajs, 2000.0 / 2.6); // 200mA / 2.6mA
  // dx_set_operating_mode(&dx, bajs, DX_POSITION_CONTROL);
  // vTaskDelay(50);
  // dx_enable_torque(&dx, bajs, true);
  // vTaskDelay(50);
  // printf("Big chungus\n");

  // while (true) {
  //   // dx_set_goal_velocity(&dx, 1, 10, false);
  //   dx_set_goal_position(&dx, bajs, 4096 / 2, false);
  //   vTaskDelay(50);
  //   dx_set_goal_position(&dx, bajs, 4096 / 2 + 50, false);
  //   vTaskDelay(50);
  // }


  // vTaskDelay(1);
  // dx_set_goal_velocity(&dx, 1, 0, false);
  // vTaskDelay(1);

  printf("Initialised");
}