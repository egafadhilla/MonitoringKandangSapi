#include <stdio.h>
#include "driver/gpio.h"
#include <driver/uart.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include <string.h>
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "DHT22.h"

static const char* TAG = "DHT_READER";
static const char* TAG_RS485 = "RS485";

uint8_t tx_buffer[50];
uint8_t rx_buffer[50];

static float latest_dht_humidity = 0.0f;
static float latest_dht_temperature = 0.0f;
static SemaphoreHandle_t sensor_data_mutex;

void RS485_SetTX(void);
void RS485_SetRX(void);

QueueHandle_t uart_event_queue;

void RS485_Send(uart_port_t uart_port,uint8_t* buf,uint16_t size)
{
    RS485_SetTX();
    uart_write_bytes(uart_port,buf,size);
    uart_wait_tx_done(uart_port,portMAX_DELAY);
    // [FIX] Beri jeda singkat untuk memastikan semua bit telah dikirim secara fisik
    // sebelum mengubah arah kembali ke RX. Ini adalah perbaikan paling penting.
    // Kita tingkatkan sedikit menjadi 5ms untuk jaminan stabilitas yang lebih tinggi.
    vTaskDelay(pdMS_TO_TICKS(5));
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
        if (xQueueReceive(uart_event_queue, (void*)&event,portMAX_DELAY) == pdTRUE) //menanyakan apakah ada pesan yang masuk
        {
            switch (event.type)
            {
                
                case UART_DATA:
                // menyimpan pesan apabila merupakan pesan serial
                //lalu menyimpanya dalam rx_buffer
                    uart_read_bytes(UART_NUM_2, rx_buffer, event.size, portMAX_DELAY); 
                    if(strncmp((char*)rx_buffer,"{PING_ENV}",10) == 0)
                    {
                        // [FIX UTAMA] Beri jeda sebelum merespons.
                        // Ini memberi waktu bagi master untuk beralih dari mode TX ke RX.
                        vTaskDelay(pdMS_TO_TICKS(10));
                        // [FIX 1] Kirim respons dengan format yang benar (diakhiri '}')
                        ESP_LOGI(TAG_RS485,"PING_RECEIVED");
                        RS485_Send(UART_NUM_2,(uint8_t*)"{ENV_CONNECT}",13);
                    }
                    // [IMPROVEMENT] Gunakan 'else if' karena sebuah perintah tidak mungkin PING dan REQ sekaligus.
                    else if(strncmp((char*)rx_buffer,"{REQ_ENV}",9) == 0){
                        // [FIX UTAMA] Beri jeda sebelum merespons.
                        vTaskDelay(pdMS_TO_TICKS(10));
                        float temp_val, hum_val; // [FIX 2] Deklarasikan variabel lokal
                        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            temp_val = latest_dht_temperature;
                            hum_val = latest_dht_humidity;
                            xSemaphoreGive(sensor_data_mutex);
                        } else {
                            // Gagal mendapatkan mutex, kirim nilai error
                            temp_val = -1.0f;
                            hum_val = -1.0f;
                        }
                        
                        char response_buffer[100];
                        snprintf(response_buffer, sizeof(response_buffer), "{\"temperature\":%.2f,\"humidity\":%.2f}", temp_val, hum_val);
                        RS485_Send(UART_NUM_2, (uint8_t*)response_buffer, strlen(response_buffer));
                        ESP_LOGI(TAG_RS485, "Sent sensor data: %s", response_buffer);
                    }

                    //ESP_LOGI(TAG_RS485,"Received : %.*s",event.size,rx_buffer);//log untuk melihat pesan yang diterima
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


void DHT_reader_task(void *pvParameter)
{
	dht22_handle_t* dht_sensor = dht22_init(GPIO_NUM_26);
            if (!dht_sensor) {
		ESP_LOGE(TAG, "Failed to initialize DHT22 sensor. Deleting task.");
		vTaskDelete(NULL);
	}

	while(1) {
		ESP_LOGI(TAG, "Reading DHT Sensor...");
		int ret = readDHT(dht_sensor);
		errorHandler(ret);

		// [FIX 1] Lindungi penulisan ke variabel global dengan mutex
		if (ret == DHT_OK) {
			if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
				latest_dht_humidity = getHumidity(dht_sensor);
				latest_dht_temperature = getTemperature(dht_sensor);
				xSemaphoreGive(sensor_data_mutex);

				// Lakukan logging setelah mutex dilepaskan
				ESP_LOGI(TAG, "Humidity: %.2f %%, Temperature: %.2f C", latest_dht_humidity, latest_dht_temperature);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

void app_main()
{
	//Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "NVS Initialized.");
	RS485_Init();
	sensor_data_mutex = xSemaphoreCreateMutex();

	xTaskCreate(&DHT_reader_task, "DHT_reader_task", 2048, NULL, 5, NULL );
	ESP_LOGI(TAG, "DHT Reader Task Started. Main task is now idle.");
	xTaskCreate(uart_event_task, "uart_event_task", 2048 * 4, NULL, 5, NULL);
	ESP_LOGI(TAG_RS485, "Setup complete. Main task is now idle.");

    
    // app_main task can now be deleted or put into an idle loop
    vTaskDelete(NULL);
}