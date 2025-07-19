#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include <driver/uart.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
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
static const char* TAG_MIC = "MICROPHONE";

uint8_t tx_buffer[50];
uint8_t rx_buffer[50];

static float latest_dht_humidity = 0.0f;
static float latest_dht_temperature = 0.0f;
static float latest_mic_db = 0.0f;
static TickType_t last_log_time = 0;

static SemaphoreHandle_t sensor_data_mutex;

// Mikrofon GY-MAX konfigurasi - GPIO 25 menggunakan ADC2
#define MIC_GPIO_PIN GPIO_NUM_25
#define MIC_ADC_CHANNEL ADC_CHANNEL_8  // GPIO 25 = ADC2_CHANNEL_8
#define MIC_ADC_ATTEN ADC_ATTEN_DB_12  // Menggunakan ADC_ATTEN_DB_12 (pengganti ADC_ATTEN_DB_11)
#define MIC_ADC_WIDTH ADC_BITWIDTH_12
#define MIC_SAMPLE_COUNT 50  // Kurangi sample count untuk performa yang lebih baik
#define MIC_SAMPLE_INTERVAL_MS 2

// ADC handle dan kalibrasi untuk mikrofon
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc2_cali_handle;
static bool adc_calibrated = false;

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

void mic_init(void)
{
    // Inisialisasi ADC2 untuk mikrofon (GPIO 25)
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    // Konfigurasi channel ADC2
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = MIC_ADC_WIDTH,
        .atten = MIC_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, MIC_ADC_CHANNEL, &config));

    // Inisialisasi kalibrasi ADC2 - menggunakan line fitting untuk ESP32
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .atten = MIC_ADC_ATTEN,
        .bitwidth = MIC_ADC_WIDTH,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc2_cali_handle);
    if (ret == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG_MIC, "ADC2 calibration successful");
    } else {
        ESP_LOGW(TAG_MIC, "ADC2 calibration failed, using raw values");
        adc_calibrated = false;
    }
    
    ESP_LOGI(TAG_MIC, "Mikrofon GY-MAX initialized on GPIO %d (ADC2)", MIC_GPIO_PIN);
}

float read_microphone_volume(void)
{
    uint32_t adc_reading = 0;
    int raw_value = 0;
    
    // Ambil sampel ADC beberapa kali untuk mendapatkan rata-rata
    for (int i = 0; i < MIC_SAMPLE_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, MIC_ADC_CHANNEL, &raw_value));
        adc_reading += raw_value;
        vTaskDelay(pdMS_TO_TICKS(MIC_SAMPLE_INTERVAL_MS));
    }
    adc_reading /= MIC_SAMPLE_COUNT;
    
    // Konversi ke voltage
    int voltage = 0;
    if (adc_calibrated) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_reading, &voltage));
    } else {
        // Gunakan konversi manual jika kalibrasi gagal
        voltage = (adc_reading * 3300) / 4095;  // Untuk ADC 12-bit
    }
    
    // Konversi ke nilai dB
    // Voltage reference untuk 0 dB (nilai ketika mikrofon tidak mendeteksi suara)
    const float voltage_ref = 1650.0f;  // 1.65V (setengah dari 3.3V sebagai referensi)
    const float voltage_min = 50.0f;    // Voltage minimum untuk mencegah log(0)
    
    // Hitung amplitudo sinyal (selisih dari reference voltage)
    float voltage_amplitude = fabs(voltage - voltage_ref);
    
    // Pastikan voltage tidak terlalu kecil untuk mencegah log negatif tak terhingga
    if (voltage_amplitude < voltage_min) {
        voltage_amplitude = voltage_min;
    }
    
    // Konversi ke dB menggunakan formula: dB = 20 * log10(V/Vref)
    // Untuk mikrofon, kita gunakan voltage_amplitude sebagai sinyal
    float db_value = 20.0f * log10f(voltage_amplitude / voltage_min);
    
    // Batasi nilai dB dalam rentang yang masuk akal untuk mikrofon
    // Rentang tipikal: 30-90 dB untuk lingkungan normal
    if (db_value < 30.0f) {
        db_value = 30.0f;  // Noise floor
    } else if (db_value > 90.0f) {
        db_value = 90.0f;  // Maximum practical level
    }
    
    return db_value;
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
                        float temp_val, hum_val, mic_val; // [FIX 2] Deklarasikan variabel lokal
                        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            temp_val = latest_dht_temperature;
                            hum_val = latest_dht_humidity;
                            mic_val = latest_mic_db;
                            xSemaphoreGive(sensor_data_mutex);
                        } else {
                            // Gagal mendapatkan mutex, kirim nilai error
                            temp_val = -1.0f;
                            hum_val = -1.0f;
                            mic_val = -1.0f;
                        }
                        
                        char response_buffer[150];
                        snprintf(response_buffer, sizeof(response_buffer), "{\"temperature\":%.2f,\"humidity\":%.2f,\"microphone\":%.2f}", temp_val, hum_val, mic_val);
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

                // Gunakan logika "millis" untuk logging setiap 2000ms
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - last_log_time) >= pdMS_TO_TICKS(2000)) {
                    ESP_LOGI(TAG, "Humidity: %.2f %%, Temperature: %.2f C", latest_dht_humidity, latest_dht_temperature);
                    last_log_time = current_time; // Perbarui waktu logging terakhir
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void microphone_reader_task(void *pvParameter)
{
    ESP_LOGI(TAG_MIC, "Starting microphone reader task");
    
    while(1) {
        float db_level = read_microphone_volume();
        
        // Update nilai mikrofon dengan proteksi mutex
        if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
            latest_mic_db = db_level;
            xSemaphoreGive(sensor_data_mutex);
        }
        
        // Log setiap 3 detik untuk mengurangi spam log
        static TickType_t last_mic_log_time = 0;
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_mic_log_time) >= pdMS_TO_TICKS(3000)) {
            ESP_LOGI(TAG_MIC, "Microphone level: %.2f dB", db_level);
            last_mic_log_time = current_time;
        }
        
        // Baca mikrofon setiap 500ms untuk respons yang lebih cepat
        vTaskDelay(pdMS_TO_TICKS(500));
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
	
    // Inisialisasi komunikasi RS485
	RS485_Init();
    
    // Inisialisasi sensor mikrofon
    mic_init();
    
    // Inisialisasi mutex untuk proteksi data sensor
	sensor_data_mutex = xSemaphoreCreateMutex();

    // Task untuk membaca sensor DHT (prioritas sedang)
	xTaskCreate(&DHT_reader_task, "DHT_reader_task", 2048, NULL, 4, NULL );
	ESP_LOGI(TAG, "DHT Reader Task Started.");
    
    // Task untuk membaca sensor mikrofon (prioritas sedang)
    xTaskCreate(&microphone_reader_task, "microphone_reader_task", 2048, NULL, 4, NULL );
    ESP_LOGI(TAG_MIC, "Microphone Reader Task Started.");
    
    // Task untuk komunikasi RS485 (prioritas tinggi untuk komunikasi)
	xTaskCreate(uart_event_task, "uart_event_task", 2048 * 4, NULL, 6, NULL);
	ESP_LOGI(TAG_RS485, "RS485 Communication Task Started with HIGH priority.");

    ESP_LOGI(TAG, "All tasks started. Setup complete.");
    
    // app_main task can now be deleted or put into an idle loop
    vTaskDelete(NULL);
}