#include <driver/uart.h>
#include <driver/gpio.h>

#include "protocol.h"
#include "hardware.h"
#include "main.h"

int uart_proper_read(int uart_num, uint8_t *buf, int len) {
    int read = 0;

    while (read < len) {
        int res = uart_read_bytes(uart_num, buf, len - read, 100);
        if (res == 0) {
            return -1;
        }
        read += res;
    }
    return 0;
}

void uart_thread() {
    uart_config_t uart_console = {
        .baud_rate = BAUD_UARD_CONSOLE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(NUM_UART_CONSOLE, &uart_console);
    uart_set_pin(NUM_UART_CONSOLE, 1, 3, -1, -1);
    uart_driver_install(NUM_UART_CONSOLE, 512, 512, 20, NULL, 0);
    while (true) {
        uint8_t id;
        int e;
        e = uart_proper_read(NUM_UART_CONSOLE, &id, 1);
        if (e) {
            continue;
        }

        int rx_len = id_to_len(id);

        uint8_t rx_buf[254];
        uint8_t tx_buf[254];
        if (rx_len) {
            e = uart_proper_read(NUM_UART_CONSOLE, rx_buf, rx_len);
            if (e) {
                continue;
            }
        }
        int tx_len = handle_message(id, rx_buf, tx_buf);
        uart_write_bytes(NUM_UART_CONSOLE, tx_buf, tx_len);
    }
}