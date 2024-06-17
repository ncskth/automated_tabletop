#include <sys/socket.h>
#include <esp_netif.h>
#include <nvs.h>
// #include <mdns.h>
#include <netdb.h>
#include <driver/spi_slave.h>
#include <errno.h>
#include <esp_event.h>
#include <esp_partition.h>
// #include <esp_ota_ops.h>
#define TCP_PORT 1337

#include "ethernet.h"
#include "main.h"
#include "protocol.h"

volatile int tcp_socket = -1;
volatile bool got_ip = false;

// returns 1 if error
bool yielding_read(int client, uint8_t *buf, size_t len) {
    size_t received = 0;
    while (received != len) {
        int bytes_received = recv(client, buf + received, len - received, MSG_DONTWAIT);

        if (bytes_received < 0 ) {
            if (errno == ERR_WOULDBLOCK) {
                vTaskDelay(10);
                continue;
            }
            if (errno == ENOTCONN) {
                return 1;
                break;
            }

            vTaskDelay(10);
            continue;
        }
        if (bytes_received > 0) {
            received += bytes_received;
            printf("got %d bytes\n", bytes_received);
        }
    }
    return 0;
}

void got_ip_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    printf("Ethernet Got IP Address\n");
    printf("~~~~~~~~~~~\n");
    printf("ETHIP:" IPSTR "\n", IP2STR(&ip_info->ip));
    printf("ETHMASK:" IPSTR "\n", IP2STR(&ip_info->netmask));
    printf("ETHGW:" IPSTR "\n", IP2STR(&ip_info->gw));
    printf("~~~~~~~~~~~\n");

    struct sockaddr_in tcpServerAddr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT)
    };
    // mdns_init();
    // char hostname[32];
    // sprintf(hostname, "airhockey");
    // mdns_hostname_set(hostname);
    // printf("mDNS hostname: %s\n", hostname);

    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    bind(tcp_socket, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr));
    listen(tcp_socket, 1);
    fcntl(tcp_socket, F_SETFL, O_NONBLOCK);
    got_ip = 1;
}

void lost_ip_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    if (got_ip) {
        close(tcp_socket);
        // mdns_free();
    }
    got_ip = 0;
}

void tcp_server_thread(void *user) {
    eth_init();

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &lost_ip_handler, NULL));


    struct sockaddr_in remote_addr;
	socklen_t socklen;
	socklen = sizeof(remote_addr);
    uint8_t rx_buf[512];
    uint8_t tx_buf[32];

    while (1) {
        vTaskDelay(10);
        if (!got_ip) {
            // printf("no wifi\n");
            vTaskDelay(100);
            continue;
        }
        int client_socket = -1;
        client_socket = accept(tcp_socket,(struct sockaddr *)&remote_addr, &socklen);

        if (client_socket < 0) {
            // printf("no client\n");
            vTaskDelay(10);
            continue;
        }
        fcntl(tcp_socket, F_SETFL, O_NONBLOCK);
        // handle client loop
        while (1) {
            uint8_t id;
            if (yielding_read(client_socket, &id, 1)) {
                break;
            }
            uint8_t out_buf[255];
            uint8_t in_buf[255];
            uint8_t len = id_to_len(id);

            if (len) {
                if (yielding_read(client_socket, in_buf, len)) {
                    break;
                }
            }
            uint8_t resp_len = handle_message(id, in_buf, out_buf);
            if (resp_len) {
                send(client_socket, out_buf, resp_len, 0);
            }
            vTaskDelay(10);
        }// handle client loop
        close(client_socket);

    }// accept client loop
}