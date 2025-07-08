
#include <stdio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_log.h>
#include <stdlib.h>
#include <time.h>


#define UART_PORT UART_NUM_1
#define TXD_PIN 16
#define RXD_PIN 17
#define RTS_PIN 5
#define BUF_SIZE 128

static const char* TAG = "RS485_SLAVE";
uint8_t rx_buffer[BUF_SIZE];

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, RTS_PIN, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    // Set RTS (DE+RE) ke LOW (mode receive) saat awal
    uart_set_rts(UART_PORT, 0);
}

void app_main(void) {
    uart_init();
    srand((unsigned)time(NULL));
    while (1) {
        // Pastikan mode receive
        uart_set_rts(UART_PORT, 0);
        int len = uart_read_bytes(UART_PORT, rx_buffer, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            rx_buffer[len] = 0;
            ESP_LOGI(TAG, "Received: %s", rx_buffer);
            if (strcmp((char*)rx_buffer, "HELLO") == 0) {
                int random_value = rand() % 101;
                char response[20];
                snprintf(response, sizeof(response), "HI:%d", random_value);
                // Ubah ke mode transmit
                uart_set_rts(UART_PORT, 1);
                uart_write_bytes(UART_PORT, response, strlen(response));
                ESP_LOGI(TAG, "Sent: %s", response);
                // Kembali ke mode receive
                uart_set_rts(UART_PORT, 0);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}