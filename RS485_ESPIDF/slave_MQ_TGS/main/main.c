#include <stdio.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_log.h>
#include <freertos/semphr.h> // Diperlukan untuk Mutex (Semaphore)
#include "esp_adc/adc_oneshot.h"

#define TGS2602_ADC_CHANNEL ADC_CHANNEL_6  // GPIO14 (ADC2)
#define MQ136_ADC_CHANNEL   ADC_CHANNEL_2  // GPIO2  (ADC2)

// 2. Gunakan konstanta untuk "magic numbers"
#define READ_INTERVAL_MS 1000

// 3. Gunakan tag yang konsisten untuk logging
static const char* TAG_SENSOR = "SENSOR_READER";
static const char* TAG_RS485 = "RS485";
uint8_t tx_buffer[50];
uint8_t rx_buffer[50];
bool gpio_state= 1;

// [FIX 3] Gunakan tipe data yang benar dan lindungi dengan mutex
static int latest_tgs_value = 0;
static int latest_mq_value = 0;
static SemaphoreHandle_t sensor_data_mutex;

QueueHandle_t uart_event_queue;
static adc_oneshot_unit_handle_t adc2_handle; // <-- [FIX 1] Pindahkan handle ke scope global/static

    void RS485_SetTX(void);
    void RS485_SetRX(void);

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

void ADC_Init(void)
{
    //inisialisasi ADC2 (One-Shot Mode)
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //konfigurasi Channel ADC2
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Menggunakan resolusi default (12-bit)
        .atten = ADC_ATTEN_DB_12,      // Menggunakan ADC_ATTEN_DB_12 yang tidak deprecated
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, TGS2602_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, MQ136_ADC_CHANNEL, &config));
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
                    if(strncmp((char*)rx_buffer,"{PING_GAS}",10) == 0)
                    {
                        // [FIX UTAMA] Beri jeda sebelum merespons.
                        // Ini memberi waktu bagi master untuk beralih dari mode TX ke RX.
                        vTaskDelay(pdMS_TO_TICKS(10));
                        // [FIX 1] Kirim respons dengan format yang benar (diakhiri '}')
                        RS485_Send(UART_NUM_2,(uint8_t*)"{GAS_CONNECT}",13);
                        ESP_LOGI(TAG_RS485,"PING_RECEIVED");
                    }
                    // [IMPROVEMENT] Gunakan 'else if' karena sebuah perintah tidak mungkin PING dan REQ sekaligus.
                    else if(strncmp((char*)rx_buffer,"{REQ_GAS}",9) == 0){
                        // [FIX UTAMA] Beri jeda sebelum merespons.
                        vTaskDelay(pdMS_TO_TICKS(10));
                        int tgs_val, mq_val;
                        // [FIX 2 & 4] Ambil data sensor dengan aman dan kirimkan
                        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            tgs_val = latest_tgs_value;
                            mq_val = latest_mq_value;
                            xSemaphoreGive(sensor_data_mutex);
                        } else {
                            // Gagal mendapatkan mutex, kirim nilai error
                            tgs_val = -1;
                            mq_val = -1;
                        }
                        
                        char response_buffer[100];
                        snprintf(response_buffer, sizeof(response_buffer), "{\"tgs2602\":%d,\"mq136\":%d}", tgs_val, mq_val);
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

void sensor_read_task(void *pvParameter)
{
    // <-- [FIX 2] Task harus berjalan dalam loop tak terbatas
    while(1) {
        int tgs2602 = 0;
        int mq136 = 0;

        // Langkah 1: Baca kedua sensor terlebih dahulu untuk mendapatkan nilai terbaru
        esp_err_t tgs_status = adc_oneshot_read(adc2_handle, TGS2602_ADC_CHANNEL, &tgs2602);
        esp_err_t mq_status = adc_oneshot_read(adc2_handle, MQ136_ADC_CHANNEL, &mq136);

        // Langkah 2 (FIX): Kunci mutex SATU KALI untuk memperbarui kedua nilai secara atomik.
        // Ini mencegah race condition di mana task lain membaca data yang setengah diperbarui.
        if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
            if (tgs_status == ESP_OK) latest_tgs_value = tgs2602;
            if (mq_status == ESP_OK) latest_mq_value = mq136;
            xSemaphoreGive(sensor_data_mutex);
        }

        // Langkah 3: Lakukan logging setelah mutex dilepaskan.
        if (tgs_status == ESP_OK) {
            ESP_LOGI(TAG_SENSOR, "TGS2602 Raw Value: %d", tgs2602);
        } else if (tgs_status == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_SENSOR, "ADC2 reading for TGS2602 timed out");
        } else {
            ESP_LOGE(TAG_SENSOR, "Failed to read TGS2602. Error: %s", esp_err_to_name(tgs_status));
        }

        if (mq_status == ESP_OK) {
            ESP_LOGI(TAG_SENSOR, "MQ136 Raw Value: %d", mq136);
        } else {
            ESP_LOGE(TAG_SENSOR, "Failed to read MQ136. Error: %s", esp_err_to_name(mq_status));
        }

        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}   

void app_main(void)
{
    RS485_Init();
    ADC_Init();
    // Buat mutex sebelum task-task yang menggunakannya dimulai
    sensor_data_mutex = xSemaphoreCreateMutex();

    xTaskCreate(uart_event_task, "uart_event_task", 2048 * 4, NULL, 5, NULL);
    xTaskCreate(sensor_read_task, "sensor_read_task", 2048 * 2, NULL, 5, NULL);

    // <-- [BEST PRACTICE] Setelah task lain dibuat, task app_main bisa dihapus
    // atau dibuat idle untuk menghemat resource.
    ESP_LOGI(TAG_RS485, "Setup complete. Main task is now idle.");
    vTaskDelete(NULL); // Hapus task app_main
}