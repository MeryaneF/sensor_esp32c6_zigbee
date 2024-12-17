#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define IR_SENSOR_PIN GPIO_NUM_11

// Estados da máquina de estados
typedef enum {
    AGUARDANDO_MOVIMENTO,
    MOVIMENTO_DETECTADO,
    MOVIMENTO_NAO_DETECTADO
} estado_t;

// Estado inicial
estado_t estado_atual = AGUARDANDO_MOVIMENTO;

// Função para configurar o GPIO do sensor IR
void configurar_gpio() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // Desativar interrupções
    io_conf.mode = GPIO_MODE_INPUT; // Configurar como entrada
    io_conf.pin_bit_mask = (1ULL << IR_SENSOR_PIN); // Máscara de pino
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // Desativar pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Ativar pull-down
    gpio_config(&io_conf);
}

// Função para gerenciar a máquina de estados
void maquina_de_estados(void *pvParameter) {
    while (1) {
        int sensor_val = gpio_get_level(IR_SENSOR_PIN);
        
        switch (estado_atual) {
            case AGUARDANDO_MOVIMENTO:
                if (sensor_val == 1) { // Movimento detectado
                    printf("Movimento detectado\n");
                    estado_atual = MOVIMENTO_DETECTADO;
                } else {
                    printf("Movimento não detectado\n");
                    estado_atual = MOVIMENTO_NAO_DETECTADO;
                }
                break;

            case MOVIMENTO_DETECTADO:
                vTaskDelay(pdMS_TO_TICKS(2000)); // Aguardar 2 segundos
                estado_atual = AGUARDANDO_MOVIMENTO;
                break;

            case MOVIMENTO_NAO_DETECTADO:
                vTaskDelay(pdMS_TO_TICKS(4000)); // Aguardar 4 segundos
                estado_atual = AGUARDANDO_MOVIMENTO;
                break;
        }
    }
}

void app_main() {
    // Configuração inicial dos GPIOs
    configurar_gpio();

    // Criar a tarefa da máquina de estados
    xTaskCreate(maquina_de_estados, "maquina_de_estados", 2048, NULL, 10, NULL);
}
