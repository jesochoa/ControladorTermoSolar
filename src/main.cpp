#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sh1106.h"
#include "encoder.h"

#define TAG "OLED_APP"

// Pines del encoder
#define ENCODER_PIN_A GPIO_NUM_7
#define ENCODER_PIN_B GPIO_NUM_10

// Configuración I2C
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_FREQ_HZ 100000 // estaba 400kHz lo pongo para trastear con el osciloscopio
#define SH1106_I2C_ADDRESS 0x3C

i2c_master_bus_handle_t bus_handle = nullptr;
i2c_master_dev_handle_t sh1106_dev_handle = nullptr;
SH1106 *oled_display = nullptr; // Instancia global del display

// Instancia del encoder
Encoder *rotary_encoder = nullptr;
Encoder *manual_enconder = nullptr;

bool init_i2c();                          // Configura I2C
bool init_gpio();                         // Configura GPIO
void pantalla();                          // Funcion imprime en pantalla
void estado_botones(bool *confirm_state); // Funcion lee estado botones
void update_encoder_display_auto();       // FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN MODO AUTOMATICO
void update_encoder_display_manual();     // FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN MODO MANUAL
void verificar_pantalla_128x64();         // Función de diagnóstico de pantalla 128x64

/************************************/
/*                                  */
/*      FUNCIÓN PRINCIPAL           */
/*                                  */
/************************************/
extern "C" void app_main()
{
    // Inicializaciones
    init_i2c();  // Inicializar I2C
    init_gpio(); // Inicializar GPIO

    // Crear instancia del display pasando el handle del dispositivo I2C
    oled_display = new SH1106(sh1106_dev_handle);

    if (oled_display->probe())
    {
        ESP_LOGI(TAG, "SH1106 128x160 detectado");
        // oled_display->init1(); // lo utilizo para borrar caracteres que me salen a la derecha

        if (oled_display->init())
        {
            ESP_LOGI(TAG, "SH1106 inicializado");
            // Limpiar pantalla
            oled_display->clear();
            oled_display->update();
        }
    }
    else
    {
        ESP_LOGE(TAG, "SH1106 no detectado");
    }

    // Crear e inicializar el encoder
    rotary_encoder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B, -1000, 1000, 10, 1, 150);
    if (!rotary_encoder->init())
    {
        ESP_LOGE(TAG, "Fallo la inicializacion del encoder");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Encoder inicializado correctamente");
    }

    manual_enconder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B, 0, 1500, 100, 10, 150);
    if (!manual_enconder->init())
    {
        ESP_LOGE(TAG, "Fallo la inicializacion del encoder");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Encoder inicializado correctamente");
    }

    pantalla(); // Mostrar pantalla inicial

    bool mode = true;                                        // Modo inicial del sistema: AUTOMATICO
    int enconder_auto_last = rotary_encoder->get_count();    // Valor inicial del encoder en modo automático
    int enconder_manual_last = manual_enconder->get_count(); // Valor inicial del encoder en modo manual
    update_encoder_display_auto();

    while (1)
    {
        // Leer botones BACK y CONFIRM
        estado_botones(&mode);

        if (mode)
        { // Modo AUTOMATICO
            rotary_encoder->update();
            // Actualizar display del encoder si hay cambios
            if (rotary_encoder->get_count() != enconder_auto_last)
            {
                update_encoder_display_auto();
                enconder_auto_last = rotary_encoder->get_count();
            }
        }
        else
        {
            // Modo MANUAL
            manual_enconder->update();
            // Actualizar display del encoder si hay cambios
            if (manual_enconder->get_count() != enconder_manual_last)
            {
                update_encoder_display_manual();
                enconder_manual_last = manual_enconder->get_count();
            }
        }
    }
}

void pantalla()
{
    // Ejemplo de función para actualizar la pantalla
    if (oled_display)
    {
        oled_display->clear();
        // 🔹 AJUSTADO: Solo 8 líneas disponibles (0-7)
        oled_display->drawString(2, 0, "  Placa Solar:");
        oled_display->drawString(2, 1, "Voltaje Placa:");
        oled_display->drawString(2, 2, " ");
        oled_display->drawString(2, 3, "Termo (W):");
        oled_display->drawString(2, 4, "  ");
        oled_display->drawString(2, 5, "Casa  (W):        ");
        oled_display->drawString(2, 6, " ");
        oled_display->drawString(2, 7, "Modo: AUTOMATICO");
        oled_display->update();
    }
}

void estado_botones(bool *confirm_state)
{
    // pulsado botón CONFIRM (GPIO6)
    if (gpio_get_level(GPIO_NUM_6) == 0)
    {
        *confirm_state = !*confirm_state;

        if (*confirm_state == 1)
        {
            oled_display->drawString(70, 3, "     ");
            oled_display->drawString(2, 5, "Casa  (W):");
            oled_display->drawString(2, 7, "Modo: AUTOMATICO");
            oled_display->update();
            update_encoder_display_auto();
        }
        else
        {
            oled_display->drawString(70, 5, "     ");
            oled_display->drawString(2, 7, "Modo: MANUAL     ");
            oled_display->update();
            update_encoder_display_manual();
        }
        // Pequeña demora para evitar rebotes
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Pulsado botón BACK (GPIO5)
    if (gpio_get_level(GPIO_NUM_5) == 0)
    {
        if (!*confirm_state)
        {
            manual_enconder->set_count(0);
        }
        else
        {
            rotary_encoder->set_count(0);
        }
    }
}

// 🔹 FUNCIÓN PARA INICIALIZAR I2C
bool init_i2c()
{
    ESP_LOGI(TAG, "Inicializando I2C...");
    esp_err_t ret;

    // 1. Configurar el bus I2C
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        }};

    // 2. Crear el bus I2C
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error creando bus I2C: %s", esp_err_to_name(ret));
        return false;
    }

    // 3. Configurar el dispositivo SH1106 en el bus I2C
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SH1106_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // 4. Agregar dispositivo al bus I2C
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &sh1106_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error agregando dispositivo I2C: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle); // Limpiar el bus
        bus_handle = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "I2C inicializado correctamente");
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa
    return true;
}

// 🔹 FUNCIÓN PARA INICIALIZAR GPIO
bool init_gpio()
{
    ESP_LOGI(TAG, "Inicializando GPIO ...");
    esp_err_t ret;

    // Configurar GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << GPIO_NUM_5) | (1ULL << GPIO_NUM_6)), // Botones BACK y CONFIRM
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configurando GPIO: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "IO inicializado correctamente");
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa
    return true;
}

//  FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN PANTALLA EN MODO AUTOMATICO
void update_encoder_display_auto()
{
    if (oled_display && rotary_encoder)
    {
        char buffer[5];
        snprintf(buffer, sizeof(buffer), "%d", rotary_encoder->get_count());
        oled_display->drawString(70, 5, "     "); // Limpiar área anterior
        oled_display->drawString(70, 5, buffer);
        oled_display->update();
    }
}

//  FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN PANTALLA EN MODO MANUAL
void update_encoder_display_manual()
{
    if (oled_display && manual_enconder)
    {
        char buffer[5];
        // itaa utiliza menos memoria que snprintf()
        itoa(manual_enconder->get_count(), buffer, 10);
        oled_display->drawString(70, 3, "     "); // Limpiar área anterior
        oled_display->drawString(70, 3, buffer);
        oled_display->update();
    }
}

void verificar_pantalla_128x64()
{
    if (!oled_display)
        return;

    oled_display->clear();

    ESP_LOGI(TAG, "Verificando SH1106 128x64");

    oled_display->drawPixel(2, 1, true);
    oled_display->drawString(50, 4, "1 OK");
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->drawPixel(129, 1, true);
    oled_display->drawString(50, 4, "2 OK");
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->drawPixel(2, 63, true);
    oled_display->drawString(50, 4, "3 OK");
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->drawPixel(129, 63, true);
    oled_display->drawString(50, 4, "4 OK");
    oled_display->update();

    for (int i = 0; i < 129; i++)
    {
        oled_display->drawPixel(i, 1, true);  // Línea superior
        oled_display->drawPixel(i, 63, true); // Línea inferior
    }
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 1; i < 64; ++i)
    {
        oled_display->drawPixel(2, i, true);   // Línea Izquierda
        oled_display->drawPixel(129, i, true); // Línea Derecha
    }
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    // oled_display->clear();

    // Texto en las 4 esquinas
    oled_display->drawString(2, 0, "2, 0"); // Sup-izq
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->drawString(100, 0, "129,0"); // Sup-der
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->drawString(2, 7, "0,63"); // Inf-izq
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->drawString(94, 7, "129,63"); // Inf-der
    oled_display->update();
    vTaskDelay(pdMS_TO_TICKS(500));
    oled_display->clear();

    // Centro
    oled_display->drawString(50, 3, "128x64");
    oled_display->drawString(50, 4, "OK");

    oled_display->update();
}