#ifndef ESP_NOW_CONFIG_H
#define ESP_NOW_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Estructura del medidor exterior
typedef struct
{
    uint32_t voltage;
    uint32_t potencia_watts;
    bool direction;
} power_data_t;

// Declarar la variable global como externa para que se comparta con el main
extern power_data_t received_power_data;

// Funciones públicas
esp_err_t init_esp_now_custom(void);

#endif // ESP_NOW_CONFIG_H