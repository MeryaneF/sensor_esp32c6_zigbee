#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_ha_standard.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


// Definindo pinos para os sensores
#define PIR_SENSOR_PIN    ADC1_CHANNEL_0  // Pino para o PIR
#define LDR_SENSOR_PIN    ADC1_CHANNEL_3  // Pino para o LDR

// Configurações do ADC para o LDR
#define NUM_READINGS      10  // Número de leituras para média

// Variáveis globais
static const char *TAG = "Dispositvo Final";
int pir_status = 0;
int ldr_status = 0;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool calibrated = false;

// Função para inicializar o driver do LED
static esp_err_t deferred_driver_init(void)
{
    // Inicializa o LED no estado desligado
    light_driver_init(LIGHT_DEFAULT_OFF);
    return ESP_OK;
}

// Callback para iniciar o comissionamento Zigbee
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

// Handler para processar sinais Zigbee
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

// Função para inicializar a calibração do ADC
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
            .bitwidth = ADC_WIDTH_BIT_12,
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
            .bitwidth = ADC_WIDTH_BIT_12,
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

// Função para finalizar a calibração do ADC
static void adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregistering Curve Fitting Calibration Scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregistering Line Fitting Calibration Scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

// Função para ler os sensores e enviar dados via Zigbee
void read_sensors()
{
    // Leitura do sensor PIR
    int pir_value = adc1_get_raw(PIR_SENSOR_PIN);

    // Leitura do sensor LDR com calibração
    int ldr_value_raw = 0;
    int ldr_voltage = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        int adc_raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_cali_handle, ADC_UNIT_1, &adc_raw));
        ldr_value_raw += adc_raw;

        if (calibrated) {
            int voltage;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
            ldr_voltage += voltage;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ldr_value_raw /= NUM_READINGS;
    ldr_voltage /= NUM_READINGS;

    // Lógica para o sensor PIR
    if (pir_value < 1000) {
        pir_status = 0;  // Sem presença detectada
        ESP_LOGI("PIR", "Nenhuma presença detectada");
    } else {
        pir_status = 1;  // Presença detectada
        ESP_LOGI("PIR", "Presença detectada");
    }

    // Lógica para o sensor LDR
    if (ldr_voltage < 1000) {
        ldr_status = 0;  // Iluminação insuficiente
        ESP_LOGI("LDR", "Iluminação insuficiente: %d mV", ldr_voltage);
    } else if (ldr_voltage >= 3000) {
        ldr_status = 1;  // Iluminação suficiente
        ESP_LOGI("LDR", "Iluminação suficiente: %d mV", ldr_voltage);
    } else {
        ldr_status = 1;  // Iluminação normal
        ESP_LOGI("LDR", "Iluminação normal: %d mV", ldr_voltage);
    }

    // Enviar os dados para o coordenador Zigbee
    ESP_LOGI(TAG, "Enviando dados via Zigbee: PIR = %d, LDR = %d", pir_status, ldr_status);
    // Função de envio real pode ser implementada aqui
}

// Tarefa principal para integração Zigbee e sensores
static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &info);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(NULL);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    while (1) {
        read_sensors();
        vTaskDelay(pdMS_TO_TICKS(5000));  // Aguarda 5 segundos antes de ler novamente
    }
}

// Função principal
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    // Inicializa o NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // Configura a plataforma Zigbee
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Configuração do ADC para os sensores
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configuração do canal do PIR
    adc_oneshot_chan_cfg_t pir_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_WIDTH_BIT_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIR_SENSOR_PIN, &pir_config));

    // Inicialização da calibração do ADC para o LDR
    calibrated = adc_calibration_init(ADC_UNIT_1, LDR_SENSOR_PIN, ADC_ATTEN_DB_12, &adc1_cali_handle);

    // Inicia a tarefa Zigbee
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
