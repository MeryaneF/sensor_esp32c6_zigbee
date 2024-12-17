/*Programa que testa o funcionamento de um sensor PIR
Feito para o teste 2 - Lógica simples com um sensor PIR utilizando o GPIO 11 do dispositivo projetado*/

// Bibliotecas necessárias 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"

#define PIR_SENSOR_GPIO    GPIO_NUM_11
#define TAG "PIR_SENSOR"

void pir_sensor_task(void* arg)
{
    esp_rom_gpio_pad_select_gpio(PIR_SENSOR_GPIO);
    gpio_set_direction(PIR_SENSOR_GPIO, GPIO_MODE_INPUT);

    while(1) {
        int pir_state = gpio_get_level(PIR_SENSOR_GPIO);
        if(pir_state) {
            ESP_LOGI(TAG, "Presença detectada");
        } else {
            ESP_LOGI(TAG, "Presença não detectada");
        }
        vTaskDelay(pdMS_TO_TICKS(3600));
    }
}

void app_main(void)
{
    xTaskCreate(pir_sensor_task, "pir_sensor_task", 2048, NULL, 10, NULL);
}
