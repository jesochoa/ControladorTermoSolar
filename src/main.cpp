#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sh1106.h"
#include "encoder.h"
#include "esp_timer.h"

#define TAG "OLED_APP"

// Pines del encoder
#define ENCODER_PIN_A GPIO_NUM_7
#define ENCODER_PIN_B GPIO_NUM_10

// Configuración I2C
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_FREQ_HZ 100000 // estaba 400kHz lo pongo para trastear con el osciloscopio
#define SH1106_I2C_ADDRESS 0x3C

#include "rom/ets_sys.h" // Para ets_delay_us
// Duración del pulso en ALTO (fijo)
#define ANCHO_PULSO_US 530
// Variable global para el tiempo de espera antes del disparo (0 a 10.000 us)
// "volatile" es vital para que el compilador sepa que esto cambia en tiempo real
volatile uint64_t tiempo_espera_us = 0;
esp_timer_handle_t timer_handle; // Handle del timer para el retardo

i2c_master_bus_handle_t bus_handle = nullptr;
i2c_master_dev_handle_t sh1106_dev_handle = nullptr;
SH1106 *oled_display = nullptr; // Instancia global del display

// Instancia del encoder
Encoder *rotary_encoder = nullptr;
Encoder *manual_enconder = nullptr;

bool init_i2c();                          // Configura I2C
bool init_gpio();                         // Configura GPIO
bool init_interrupts();                   // Configura interrupciones GPIO
void IRAM_ATTR isr_paso_cero(void *arg);  // Manejador de interrupciones paso por cero
void pantalla();                          // Funcion imprime en pantalla
void estado_botones(bool *confirm_state); // Funcion lee estado botones
void update_encoder_display_auto();       // FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN MODO AUTOMATICO
void update_encoder_display_manual();     // FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN MODO MANUAL
void verificar_pantalla_128x64();         // Función de diagnóstico de pantalla 128x64
void timer_callback(void *arg);           // Callback del timer de retardo

/************************************/
/*                                  */
/*      FUNCIÓN PRINCIPAL           */
/*                                  */
/************************************/
extern "C" void app_main()
{
    // Variable para el modo (automático/manual)
    bool mode = true;         // true = automático, false = manual
    int enconder_auto_last;   // Valor del Encoder en modo automático
    int enconder_manual_last; // Valor del encoder en modo manual

    // Inicializaciones
    init_i2c();        // Inicializar I2C
    init_gpio();       // Inicializar GPIO
    init_interrupts(); // Inicializar interrupciones GPIO_0 paso por cero

    // Instalar el servicio de manejo de interrupciones GPIO_1
    gpio_install_isr_service(0);

    // Asociar la función de manejo de interrupciones al pin GPIO_1
    gpio_isr_handler_add(GPIO_NUM_1, isr_paso_cero, &mode);

    // Crear el Timer de Alta Resolución
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,       // Función callback del timer
        .arg = nullptr,                    // No se pasan argumentos
        .dispatch_method = ESP_TIMER_TASK, // Ejecutar en tarea de alta prioridad (más seguro para 530us)
        .name = "disparador_pulso_retardo"};

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));

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

    enconder_auto_last = rotary_encoder->get_count();    // Valor inicial del encoder en modo automático
    enconder_manual_last = manual_enconder->get_count(); // Valor inicial del encoder en modo manual
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
                tiempo_espera_us = 10000.0f - ((float) enconder_manual_last * 6.666667f);
                //printf("Retardo manual ajustado a: %llu us\n", tiempo_espera_us);
            }
        }
        vTaskDelay(1); // Pequeña demora para evitar uso excesivo de CPU
    }
}

// Callback del Timer: Genera el pulso de 530us
void timer_callback(void *arg)
{
    if (tiempo_espera_us == 0 || tiempo_espera_us > 9999)
    {
        // Si el tiempo de espera es 0, no hacemos nada
        return;
    }

    // 1. Subir el pin (Inicio del pulso)
    gpio_set_level(GPIO_NUM_2, 1);

    // 2. Esperar exactamente 530 microsegundos
    // Bloqueamos la CPU brevemente para garantizar precisión absoluta
    ets_delay_us(ANCHO_PULSO_US);

    // 3. Bajar el pin (Fin del pulso)
    gpio_set_level(GPIO_NUM_2, 0);
}

// Función para actualizar la pantalla
void pantalla()
{
    if (oled_display)
    {
        oled_display->clear();
        //   AJUSTADO: Solo 8 líneas disponibles (0-7)
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

// FUNCIÓN PARA INICIALIZAR I2C
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

    // Configurar GPIO  BACK y CONFIRM como entradas
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
        ESP_LOGE(TAG, "Error configurando GPIO_5 Y GPIO_6: %s", esp_err_to_name(ret));
        return false;
    }

    // Configurar GPIO_2 como salida para el modulo SRC
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_2); // Pin GPIO2 para salida
    io_conf.mode = GPIO_MODE_OUTPUT;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configurando GPIO_2: %s", esp_err_to_name(ret));
        return false;
    }
    gpio_set_level(GPIO_NUM_2, 0); // Inicialmente en bajo

    ESP_LOGI(TAG, "GPIO inicializado correctamente");
    return true;
}

// Esta es la función que se ejecutará (ISR Handler) para paso por cero
void IRAM_ATTR isr_paso_cero(void *arg)
{
    // Paso de seguridad: Si ya había un timer corriendo (porque llegaron pulsos muy rápido),
    // lo detenemos para reiniciar la cuenta. Esto hace el sistema "re-disparable".
    esp_timer_stop(timer_handle);

    if (tiempo_espera_us > 0)
    {
        // Si hay retardo, programamos el timer
        esp_timer_start_once(timer_handle, tiempo_espera_us);
    }
    else
    {
        // CASO ESPECIAL: Si el tiempo es 0, no esperamos.
        // Llamamos directamente al callback o lanzamos el timer con 0.
        // Lanzarlo con 0 es más seguro para no bloquear esta ISR con los 530us del pulso.
        esp_timer_start_once(timer_handle, 0);
    }
}

// Funcion para incializar el GPIO con interrupcion
bool init_interrupts()
{
    ESP_LOGI(TAG, "Inicializando interrupciones GPIO ...");
    esp_err_t ret;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_1), // Pin GPIO1 para interrupciones
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE, // Interrupción en flanco de subida y bajada
    };
    ret = gpio_config(&io_conf);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configurando GPIO: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "IO inicializado correctamente");
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