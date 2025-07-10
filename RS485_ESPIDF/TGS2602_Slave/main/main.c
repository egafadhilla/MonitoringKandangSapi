

#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h" // Menggunakan driver ADC One-Shot yang baru

// --- Saran Perbaikan ---
// 1. Pindahkan sensor ke pin ADC1 jika berencana menggunakan Wi-Fi.
//    Contoh pin ADC1: GPIO32, GPIO33, GPIO34, GPIO35, GPIO36, GPIO39.
//    Jika tetap menggunakan ADC2, Wi-Fi tidak dapat diaktifkan.
#define TGS2602_ADC_CHANNEL ADC_CHANNEL_6  // GPIO14 (ADC2)
#define MQ136_ADC_CHANNEL   ADC_CHANNEL_2  // GPIO2  (ADC2)

// 2. Gunakan konstanta untuk "magic numbers"
#define READ_INTERVAL_MS 1000

// 3. Gunakan tag yang konsisten untuk logging
static const char *TAG = "SENSOR_READER";

void app_main(void)
{
    //------------- Inisialisasi ADC2 (One-Shot Mode)---------------//
    adc_oneshot_unit_handle_t adc2_handle;
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //------------- Konfigurasi Channel ADC2 ---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Menggunakan resolusi default (12-bit)
        .atten = ADC_ATTEN_DB_12,      // Menggunakan ADC_ATTEN_DB_12 yang tidak deprecated
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, TGS2602_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, MQ136_ADC_CHANNEL, &config));

    while (1) {
        int tgs2602 = 0;
        int mq136 = 0;

        // Membaca sensor menggunakan driver one-shot
        esp_err_t tgs_status = adc_oneshot_read(adc2_handle, TGS2602_ADC_CHANNEL, &tgs2602);
        if (tgs_status == ESP_OK) {
            ESP_LOGI(TAG, "TGS2602 Raw Value: %d", tgs2602);
        } else {
            // Peringatan: Jika Wi-Fi aktif, pembacaan ADC2 akan gagal dengan error ESP_ERR_TIMEOUT.
            if (tgs_status == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "ADC2 reading for TGS2602 timed out. Wi-Fi might be active.");
            } else {
                ESP_LOGE(TAG, "Failed to read TGS2602. Error: %s", esp_err_to_name(tgs_status));
            }
        }

        esp_err_t mq_status = adc_oneshot_read(adc2_handle, MQ136_ADC_CHANNEL, &mq136);
        if (mq_status == ESP_OK) {
            ESP_LOGI(TAG, "MQ136 Raw Value: %d", mq136);
        } else {
            ESP_LOGE(TAG, "Failed to read MQ136. Error: %s", esp_err_to_name(mq_status));
        }

        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}