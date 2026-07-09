#include "esp_now_config.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_NOW = "ESP_NOW_MODULE";

// Definición real de la variable global
power_data_t received_power_data;

// Callback privado que se ejecuta al recibir datos
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(power_data_t)) {
        memcpy(&received_power_data, incomingData, sizeof(received_power_data));
        // ESP_LOGI(TAG_NOW, "Potencia recibida: %lu W", received_power_data.potencia_watts);
    } else {
        ESP_LOGW(TAG_NOW, "Tamaño de datos ESP-NOW incorrecto.");
    }
}

// Función encargada de inicializar WiFi y ESP-NOW
esp_err_t init_esp_now_custom(void)
{
    // Inicializar el WIFI en modo Estación (STA)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        ESP_LOGE(TAG_NOW, "Error al inicializar ESP-NOW");
        return ESP_FAIL;
    }

    // Registrar callback de recepción
    return esp_now_register_recv_cb(on_data_recv);
}