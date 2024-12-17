#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const static char *TAG = "ADC_LUX_CONVERSION";

// Configurações do ADC
#define ADC_CHANNEL          ADC_CHANNEL_0  // Canal do ADC
#define ADC_ATTENUATION      ADC_ATTEN_DB_12  // Atenuação de 12 dB
#define ADC_UNIT             ADC_UNIT_1     // Unidade ADC1
#define ADC_BIT_WIDTH        ADC_BITWIDTH_DEFAULT

// Número de amostras
#define NUM_SAMPLES          28

// Dados de calibração do luxímetro e do ADC em milivolts (mV)
static const int adc_voltage_samples[NUM_SAMPLES] = {
    0, 0, 0, 0, 0, 16, 31, 47, 108, 186, 265, 389, 482, 622, 731,
    840, 949, 1088, 1166, 1274, 1351, 1445, 1507, 1554, 1615, 1677, 1709, 1756
};

static const int lux_samples[NUM_SAMPLES] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 5, 9, 12, 18, 22, 29, 36, 44, 52, 
    65, 72, 88, 101, 118, 133, 159
};

// Funções de calibração e conversão
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
int interpolate_lux(int voltage);

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

    int previous_lux = -1; // Valor inicial para a histerese

    if (calibrated) {
        while (1) {
            int adc_raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw));
            ESP_LOGI(TAG, "ADC Raw Data: %d", adc_raw);

            int voltage;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
            ESP_LOGI(TAG, "Calibrated Voltage: %d mV", voltage);

            // Converte a leitura de tensão para lux
            int lux = interpolate_lux(voltage);
            ESP_LOGI(TAG, "Lux: %d", lux);

            // Envia a mensagem conforme os níveis de lux, com histerese
            if (lux >= 30 && previous_lux < 30) {
                ESP_LOGI(TAG, "Luz suficiente");
                previous_lux = lux;
            } else if (lux < 10 && previous_lux >= 10) {
                ESP_LOGI(TAG, "Luz insuficiente");
                previous_lux = lux;
            }

            vTaskDelay(pdMS_TO_TICKS(3000));  // Delay de 3 segundos entre leituras
        }
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

// Função de interpolação linear para calcular o valor de lux
int interpolate_lux(int voltage) {
    for (int i = 0; i < NUM_SAMPLES - 1; i++) {
        if (voltage >= adc_voltage_samples[i] && voltage <= adc_voltage_samples[i + 1]) {
            int x0 = adc_voltage_samples[i];
            int x1 = adc_voltage_samples[i + 1];
            int y0 = lux_samples[i];
            int y1 = lux_samples[i + 1];

            // Fórmula de interpolação linear
            int lux = y0 + (voltage - x0) * (y1 - y0) / (x1 - x0);
            return lux;
        }
    }
    // Retorna -1 caso a tensão esteja fora do intervalo das amostras
    return -1;
}
