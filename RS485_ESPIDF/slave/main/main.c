// ESP-IDF RS485 Slave - Ping Response Example
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define RS485_TXD_PIN (17)
#define RS485_RXD_PIN (16)
#define RS485_RTS_PIN (18)
#define RS485_UART_PORT_NUM      UART_NUM_1
#define RS485_BAUD_RATE          9600
#define BUF_SIZE                 128

static const char *TAG = "RS485_SLAVE";

void app_main(void)
{
    const int uart_num = RS485_UART_PORT_NUM;
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, RS485_TXD_PIN, RS485_RXD_PIN, RS485_RTS_PIN, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);

    uint8_t rx_buf[BUF_SIZE];
    srand((unsigned int)time(NULL));

    while (1) {
        int len = uart_read_bytes(uart_num, rx_buf, BUF_SIZE-1, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            rx_buf[len] = '\0';
            ESP_LOGI(TAG, "Received: %s", rx_buf);
            if (strcmp((char*)rx_buf, "PING") == 0) {
                // Kirim balasan nilai acak
                int random_val = rand() % 1000;
                char reply[32];
                snprintf(reply, sizeof(reply), "PONG:%d", random_val);
                uart_write_bytes(uart_num, reply, strlen(reply));
                ESP_LOGI(TAG, "Sent reply: %s", reply);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
