#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const static char *TAG = "ADC_CALIBRATION_EXAMPLE";

// Configurações do ADC
#define ADC_CHANNEL          ADC_CHANNEL_0  // Canal do ADC
#define ADC_ATTENUATION      ADC_ATTEN_DB_12  // Atenuação de 12 dB
#define ADC_UNIT             ADC_UNIT_1     // Unidade ADC1
#define ADC_BIT_WIDTH        ADC_BITWIDTH_DEFAULT
#define NUM_READINGS         10  // Número de leituras para média

// Funções de calibração
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

void app_main(void) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configuração do canal do ADC1
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));

    // Inicialização da calibração do ADC1 canal 0
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool calibrated = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTENUATION, &adc1_cali_handle);

    while (1) {
        int adc_raw_sum = 0;
        int voltage_sum = 0;

        for (int i = 0; i < NUM_READINGS; i++) {
            int adc_raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw));
            adc_raw_sum += adc_raw;

            if (calibrated) {
                int voltage;
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
                voltage_sum += voltage;
            }
            vTaskDelay(pdMS_TO_TICKS(100));  // Delay curto entre as leituras
        }

        int adc_raw_avg = adc_raw_sum / NUM_READINGS;
        int voltage_avg = voltage_sum / NUM_READINGS;

        ESP_LOGI(TAG, "Average ADC Raw Data: %d", adc_raw_avg);
        if (calibrated) {
            ESP_LOGI(TAG, "Average Calibrated Voltage: %d mV", voltage_avg);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));  // Delay de 3 segundos entre leituras
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (calibrated) {
        adc_calibration_deinit(adc1_cali_handle);
    }
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Using Curve Fitting Calibration Scheme");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BIT_WIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Using Line Fitting Calibration Scheme");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BIT_WIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else {
        ESP_LOGW(TAG, "Calibration failed or not supported.");
    }
    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregistering Curve Fitting Calibration Scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregistering Line Fitting Calibration Scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
