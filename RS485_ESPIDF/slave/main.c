// ESP-IDF RS485 Slave Example
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define RS485_TXD_PIN (17)
#define RS485_RXD_PIN (16)
#define RS485_RTS_PIN (18)
#define RS485_UART_PORT_NUM      UART_NUM_1
#define RS485_BAUD_RATE          9600
#define BUF_SIZE                 128

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

    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI("RS485_SLAVE", "Received: %s", data);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
