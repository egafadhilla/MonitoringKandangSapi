#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_err.h"
#include "soc/adc_channel.h"

static const char *TAG = "GY_MAX4466_MIC";

// Define constants for ADC Continuous Mode
#define ADC_SAMPLES_COUNT 1024             // Increased buffer size for continuous mode
#define MIC_ADC_CHANNEL ADC_CHANNEL_6      // GPIO34 = ADC1_CHANNEL_6 (input only, ideal for sensors)
#define MIC_ADC_UNIT ADC_UNIT_1            // Changed to ADC1 for continuous mode support
#define SAMPLE_RATE_HZ 500               // Reduced to 500Hz (definitely within ESP32 ADC continuous limits)
#define READ_LEN 512                      // Number of samples to read per burst
#define LOG_INTERVAL_MS 500               // Log setiap 500ms
#define CONV_FRAME_SIZE 1024              // DMA frame size

// Audio Sampler structure for ADC Continuous Mode
typedef struct {
    adc_continuous_handle_t adc_handle;
    adc_oneshot_unit_handle_t adc_oneshot_handle;  // Added for fallback mode
    adc_cali_handle_t adc_cali_handle;
    int *currentAudioBuffer;
    size_t audioBufferPos;
    bool calibration_enable;
    bool oneshot_mode;  // Flag to indicate which mode is active
    TaskHandle_t taskHandle;
} AudioSampler;

// Global variables
static AudioSampler sampler;
static int audio_buffer[ADC_SAMPLES_COUNT];
static uint8_t adc_raw_buffer[CONV_FRAME_SIZE];

// ADC Continuous callback function
static bool IRAM_ATTR adc_continuous_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify task that new data is available
    vTaskNotifyGiveFromISR(sampler.taskHandle, &mustYield);
    return (mustYield == pdTRUE);
}

void microphoneTask(void *param)
{
    AudioSampler *sampler = (AudioSampler *)param;
    ESP_LOGI(TAG, "Microphone continuous reading task started");
    
    // Add task ke watchdog
    esp_task_wdt_add(NULL);
    
    uint32_t buffer_count = 0;
    uint32_t total_samples = 0;
    
    // Variables untuk tracking data terbaru untuk logging
    int32_t latest_average = 0;
    int32_t latest_amplitude = 0;
    int latest_min = 4095, latest_max = 0;
    int latest_voltage_mv = 0;
    bool new_data_available = false;
    
    // Timer untuk logging setiap 500ms
    TickType_t last_log_time = xTaskGetTickCount();
    
    // Start ADC continuous reading
    esp_err_t ret = adc_continuous_start(sampler->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC continuous: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "ADC continuous mode started successfully");
    
    while (true) {
        // Wait for notification from ADC callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        uint32_t ret_num = 0;
        ret = adc_continuous_read(sampler->adc_handle, adc_raw_buffer, CONV_FRAME_SIZE, &ret_num, 0);
        
        if (ret == ESP_OK) {
            total_samples += ret_num / sizeof(adc_digi_output_data_t);
            
            // Process the received data
            int32_t sum = 0;
            int min_val = 4095, max_val = 0;
            int sample_count = 0;
            
            for (int i = 0; i < ret_num; i += sizeof(adc_digi_output_data_t)) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_raw_buffer[i];
                
                if (p->type1.channel == MIC_ADC_CHANNEL) {
                    int adc_val = p->type1.data;
                    
                    // Store in circular buffer
                    if (sampler->audioBufferPos < ADC_SAMPLES_COUNT) {
                        sampler->currentAudioBuffer[sampler->audioBufferPos++] = adc_val;
                    }
                    
                    // Calculate running statistics
                    sum += adc_val;
                    if (adc_val < min_val) min_val = adc_val;
                    if (adc_val > max_val) max_val = adc_val;
                    sample_count++;
                    
                    // Check if buffer is full
                    if (sampler->audioBufferPos >= ADC_SAMPLES_COUNT) {
                        buffer_count++;
                        
                        // Calculate final statistics
                        latest_average = sum / sample_count;
                        latest_amplitude = max_val - min_val;
                        latest_min = min_val;
                        latest_max = max_val;
                        new_data_available = true;
                        
                        // Convert to voltage if calibration is available
                        if (sampler->calibration_enable && sampler->adc_cali_handle) {
                            esp_err_t cali_ret = adc_cali_raw_to_voltage(sampler->adc_cali_handle, latest_average, &latest_voltage_mv);
                            if (cali_ret != ESP_OK) {
                                latest_voltage_mv = 0;
                            }
                        }
                        
                        // Reset buffer
                        sampler->audioBufferPos = 0;
                        sum = 0;
                        min_val = 4095;
                        max_val = 0;
                        sample_count = 0;
                    }
                }
            }
            
            // Reset watchdog periodically
            esp_task_wdt_reset();
            
        } else if (ret == ESP_ERR_TIMEOUT) {
            // This is normal, just continue
        } else {
            ESP_LOGE(TAG, "ADC continuous read failed: %s", esp_err_to_name(ret));
        }
        
        // Check logging interval
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_log_time) >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
            if (new_data_available && buffer_count > 0) {
                if (sampler->calibration_enable && latest_voltage_mv > 0) {
                    ESP_LOGI(TAG, "Continuous | Buf#%lu | ADC:%ld | %dmV | Amp:%ld | Min:%d | Max:%d | Samples:%lu", 
                            buffer_count, latest_average, latest_voltage_mv, latest_amplitude, 
                            latest_min, latest_max, total_samples);
                } else {
                    ESP_LOGI(TAG, "Continuous | Buf#%lu | ADC:%ld | Amp:%ld | Min:%d | Max:%d | Samples:%lu", 
                            buffer_count, latest_average, latest_amplitude, latest_min, latest_max, total_samples);
                }
                new_data_available = false;
            } else {
                ESP_LOGI(TAG, "Continuous | Collecting samples... Progress: %zu/%d | Total samples: %lu", 
                        sampler->audioBufferPos, ADC_SAMPLES_COUNT, total_samples);
            }
            last_log_time = current_time;
        }
    }
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration success");
    } else {
        ESP_LOGW(TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
    }
    return calibrated;
}

// Oneshot ADC task function (fallback mode)
void microphoneTaskOneshot(void *pvParameters)
{
    TickType_t lastLogTime = xTaskGetTickCount();
    int adc_raw = 0;
    int voltage = 0;
    int amplified_value = 0;
    
    ESP_LOGI(TAG, "Microphone task started with ADC oneshot mode");
    
    // Add task to watchdog
    esp_task_wdt_add(NULL);
    
    while (1) {
        // Read ADC value
        esp_err_t ret = adc_oneshot_read(sampler.adc_oneshot_handle, MIC_ADC_CHANNEL, &adc_raw);
        if (ret == ESP_OK) {
            // Convert to voltage if calibration available
            if (sampler.calibration_enable) {
                adc_cali_raw_to_voltage(sampler.adc_cali_handle, adc_raw, &voltage);
                amplified_value = voltage * 2; // Simple amplification
            } else {
                voltage = (adc_raw * 3300) / 4095; // Manual calculation for 12-bit ADC
                amplified_value = voltage * 2;
            }
            
            // Log every 500ms
            TickType_t currentTime = xTaskGetTickCount();
            if ((currentTime - lastLogTime) >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
                ESP_LOGI(TAG, "Oneshot | ADC:%d | %dmV | Amp:%d", adc_raw, voltage, amplified_value);
                lastLogTime = currentTime;
            }
        } else {
            ESP_LOGE(TAG, "ADC oneshot read failed: %s", esp_err_to_name(ret));
        }
        
        // Reset watchdog setiap iterasi
        esp_task_wdt_reset();
        
        // Delay untuk simulate sampling rate dan memberikan waktu untuk task lain
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting GY-MAX4466 microphone with ADC Continuous Mode");
    
    // Debug info untuk memastikan konstanta yang benar
    ESP_LOGI(TAG, "Configuration: Sample Rate=%d Hz, Buffer Size=%d, Conv Frame=%d, Log Interval=%d ms", 
             SAMPLE_RATE_HZ, ADC_SAMPLES_COUNT, CONV_FRAME_SIZE, LOG_INTERVAL_MS);
    ESP_LOGI(TAG, "Using GPIO34 (ADC1_CHANNEL_6) with Continuous Mode for better performance");
    
    // Configure task watchdog untuk lebih toleran
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 15000,  // 15 detik timeout
        .idle_core_mask = (1 << 0) | (1 << 1),  // Monitor kedua core
        .trigger_panic = false,  // Jangan panic, hanya warning
    };
    esp_err_t wdt_ret = esp_task_wdt_reconfigure(&twdt_config);
    if (wdt_ret == ESP_OK) {
        ESP_LOGI(TAG, "Task watchdog reconfigured: timeout=15s, panic=false");
    } else {
        ESP_LOGW(TAG, "Failed to reconfigure watchdog: %s", esp_err_to_name(wdt_ret));
    }
            
    // Initialize the sampler structure
    sampler.currentAudioBuffer = audio_buffer;
    sampler.audioBufferPos = 0;
    sampler.calibration_enable = false;
    
    // ADC continuous configuration
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = CONV_FRAME_SIZE * 2,  // Reduced buffer size for stability
        .conv_frame_size = CONV_FRAME_SIZE,
    };
    
    esp_err_t ret = adc_continuous_new_handle(&adc_config, &sampler.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC continuous handle: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure ADC channel
    adc_digi_pattern_config_t adc_pattern[1] = {
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = MIC_ADC_CHANNEL,
            .unit = MIC_ADC_UNIT,
            .bit_width = ADC_BITWIDTH_12,  // Explicitly set to 12-bit
        },
    };
    
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,  // Changed to ADC1 for continuous mode
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    
    ret = adc_continuous_config(sampler.adc_handle, &dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC continuous: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "ADC continuous mode failed with sample rate %d Hz", SAMPLE_RATE_HZ);
        ESP_LOGE(TAG, "Falling back to ADC oneshot mode...");
        
        // Cleanup continuous mode handle
        adc_continuous_deinit(sampler.adc_handle);
        sampler.adc_handle = NULL;
        
        // Initialize ADC oneshot as fallback
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        adc_oneshot_unit_handle_t adc1_handle;
        ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC oneshot unit: %s", esp_err_to_name(ret));
            return;
        }
        
        adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = ADC_ATTEN_DB_12,
        };
        ret = adc_oneshot_config_channel(adc1_handle, MIC_ADC_CHANNEL, &config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure ADC oneshot channel: %s", esp_err_to_name(ret));
            return;
        }
        
        ESP_LOGI(TAG, "ADC oneshot mode configured successfully");
        ESP_LOGI(TAG, "Starting microphone task with oneshot ADC");
        
        // Initialize ADC calibration for oneshot mode
        sampler.calibration_enable = adc_calibration_init(MIC_ADC_UNIT, MIC_ADC_CHANNEL, ADC_ATTEN_DB_12, &sampler.adc_cali_handle);
        
        // Create task with oneshot mode
        sampler.adc_oneshot_handle = adc1_handle;
        sampler.oneshot_mode = true;
        
        BaseType_t task_ret = xTaskCreatePinnedToCore(
            microphoneTaskOneshot, 
            "microphone_task", 
            4096, 
            NULL, 
            4,  // Lower priority than continuous mode (was 5, now 4)
            NULL, 
            1   // Use Core 1 to avoid interference with system tasks on Core 0
        );
        
        if (task_ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create microphone oneshot task");
            return;
        }
        
        ESP_LOGI(TAG, "Microphone oneshot task created on Core 1 with priority 4");
        return;
    }
    
    // Initialize ADC calibration
    sampler.calibration_enable = adc_calibration_init(MIC_ADC_UNIT, MIC_ADC_CHANNEL, ADC_ATTEN_DB_12, &sampler.adc_cali_handle);
    
    // Register callback
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_continuous_callback,
    };
    ret = adc_continuous_register_event_callbacks(sampler.adc_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ADC callback: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "ADC continuous mode initialized successfully");

    // Start microphone task
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        microphoneTask, 
        "Microphone Continuous Task", 
        8192,  // Increased stack size for continuous mode
        &sampler, 
        5,     // Higher priority for real-time processing
        &sampler.taskHandle, 
        1      // Core 1 for processing
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create microphone task");
        return;
    }
    
    ESP_LOGI(TAG, "Microphone continuous task created successfully");
}