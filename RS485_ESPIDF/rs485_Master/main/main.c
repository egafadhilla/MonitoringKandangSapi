#include <stdio.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <esp_log.h>

static const char* TAG_RS485 = "RS485";
uint8_t tx_buffer[50];
uint8_t rx_buffer[50];
bool gpio_state= 1;
QueueHandle_t uart_event_queue;

   void RS485_SetTX(void);
    void RS485_SetRX(void);

void RS485_Send(uart_port_t uart_port,uint8_t* buf,uint16_t size)
{
    RS485_SetTX();
    uart_write_bytes(uart_port,buf,size);
    uart_wait_tx_done(uart_port,portMAX_DELAY);
    RS485_SetRX();
}

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
        };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2,17,16,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024 * 2, 1024 * 2, 30, &uart_event_queue, 0);
    
}

void RS485_Init(void)
{
    uart_init();
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4,GPIO_MODE_OUTPUT);

}
void RS485_SetTX()
{
    gpio_set_level(GPIO_NUM_4,1);
}
void RS485_SetRX()
{
    gpio_set_level(GPIO_NUM_4,0);
}
void uart_event_task(void *pvParameter)
{

    uart_event_t event;
    RS485_SetRX();
    while (1)
    {
        if (xQueueReceive(uart_event_queue, (void*)&event,portMAX_DELAY) == pdTRUE)
        {
            switch (event.type)
            {
                
                case UART_DATA:
                    uart_read_bytes(UART_NUM_2, rx_buffer, event.size, portMAX_DELAY);
                    if(strncmp((char*)rx_buffer,"{helloo}",8) == 0)
                    {
                        ESP_LOGI(TAG_RS485,"MOOO");
                    }
                    ESP_LOGI(TAG_RS485,"Received : %.*s",event.size,rx_buffer);
                    memset(rx_buffer,0,sizeof(rx_buffer));
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG_RS485,"UART_FRAME_ERR");
                    break;
                    default:break;
            }     
        }
    }

}

void app_main(void)
{
    RS485_Init();
    xTaskCreate(uart_event_task, "uart_event_task", 2048 * 4, NULL, 5, NULL);
    while(1)
    {
        RS485_Send(UART_NUM_2,(uint8_t*)"{helosl}",8);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
  
}