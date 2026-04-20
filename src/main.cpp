#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sh1106.h"
#include "encoder.h"
#include "esp_timer.h"
#include "driver/uart.h"     // Para comunicación Modbus RTU
#include "freertos/task.h"   // Para leer PZEM-004T periódicamente
#include "freertos/semphr.h" // Librería para usar semáforos
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

// Estructura del medidor exterior
typedef struct
{
    uint32_t voltage;
    uint32_t potencia_watts;
    bool     direction;
} power_data_t;

power_data_t received_power_data;

// Callback de ESP-NOW que se ejecuta al recibir datos
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    // Copiar los datos recibidos a nuestra estructura local
    memcpy(&received_power_data, incomingData, sizeof(received_power_data));
    ESP_LOGI("Receptor", "Potencia recibida: %lu W", received_power_data.potencia_watts);
    ESP_LOGI("Receptor","Voltage recibido: %lu V",received_power_data.voltage);
    ESP_LOGI("Receptor", "Direccion: %s", received_power_data.direction ? "ENTRANTE" : "SALIENTE");
}

#define TAG "OLED_APP"

// Configuración UART para Modbus RTU
#define UART_MODBUS_NUM UART_NUM_1
#define MODBUS_BAUDRATE 9600
#define MODBUS_RX_PIN GPIO_NUM_4
#define MODBUS_TX_PIN GPIO_NUM_3
#define MODBUS_STOP_BITS UART_STOP_BITS_1
#define MODBUS_RTS_PIN UART_PIN_NO_CHANGE
#define MODBUS_CTS_PIN UART_PIN_NO_CHANGE
#define MODBUS_RX_BUFFER_SIZE 256
#define MODBUS_TX_BUFFER_SIZE 256
#define MODBUS_SLAVE_ADDR 0x01 // Parámetros Modbus PZEM-004T 0xF8 Dirección por defecto
#define MODBUS_RESPONSE_TIMEOUT_MS 200
#define MODBUS_READ 0x04       // Función Modbus para leer registros de entrada
#define MODBUS_DIR_READ 0x0000 // Dirección inicial para leer registros de entrada

// Definiciones de pines
#define ENCODER_PIN_A GPIO_NUM_6
#define ENCODER_PIN_B GPIO_NUM_7
#define PASO_CERO GPIO_NUM_1
#define SCR GPIO_NUM_21
#define BOTON_BACK GPIO_NUM_5
#define BOTON_CONFIRM GPIO_NUM_20
#define BOTON_PUSH GPIO_NUM_8

// Configuración I2C
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_10
#define I2C_MASTER_FREQ_HZ 100000
#define SH1106_I2C_ADDRESS 0x3C
i2c_master_bus_handle_t bus_handle = nullptr;
i2c_master_dev_handle_t sh1106_dev_handle = nullptr;
SH1106 *oled_display = nullptr; // Instancia global del display

// Configuración Timer y retardo
#include "rom/ets_sys.h" // Para ets_delay_us

// El ancho del pulso de disparo en microsegundos son 500us y 5V en el modulo SRC
#define ANCHO_PULSO_US 30              // pulso de ≥20–100 µs para que la probabilidad de disparo sea alta para 5mA
#define MINIMA_POTENCIA_PLACA 50.0f    // Si la potencia de la placa es menor que este valor, no dispara el SRC
#define POTENCIA_SEGURIDAD_SCR 8400.0f // 8400us son 38W en mis pruebas son 29W, dejo un marjen seguridad

// Variable global para el tiempo de espera antes del disparo (0 a 10.000 us)
volatile uint64_t tiempo_espera_us = 0; // "volatile" es para que el compilador sepa que esto cambia en tiempo real

SemaphoreHandle_t oled_mutex = NULL; // Creo Mutex para proteger el acceso al OLED

esp_timer_handle_t timer_handle; // Handle del timer para el retardo

// Modo inicial: true = AUTOMATICO, false = MANUAL
bool mode = true;

// Instancia del encoder
Encoder *rotary_encoder = nullptr;
Encoder *manual_enconder = nullptr;

// Estructura para datos PZEM-004T
typedef struct
{
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float power_factor;
    uint16_t alarms;
} pzem_data_t;

bool init_i2c();
bool init_gpio();
bool init_interrupts(void *arg);
void isr_paso_cero(void *arg);
void pantalla();
void estado_botones();
void update_encoder_display_auto(pzem_data_t *datos);
void update_encoder_display_manual();
void timer_init();
void timer_callback(void *arg);                     // Callback del timer de retardo
void leer_pzem_004(void *arg);                      // Tarea para leer datos del PZEM-004T
static void init_modbus_uart(void);                 // Inicializar UART para Modbus
static esp_err_t read_pzem_data(pzem_data_t *data); // Leer datos del PZEM-004T
static uint64_t mapear_potencia_a_retardo(float potencia);

// Leer respuesta Modbus
static esp_err_t read_modbus_response(uint8_t *buffer, uint16_t *length, uint16_t max_length);

// Enviar comando Modbus RTU
static esp_err_t send_modbus_command(uint8_t slave_addr, uint8_t function_code,
                                     uint16_t start_addr, uint16_t reg_count);
// Función para calcular CRC16 Modbus
static uint16_t calculate_crc16(const uint8_t *data, uint16_t length);
// Verificar CRC de respuesta
static bool verify_crc(const uint8_t *data, uint16_t length);

/******************************************************************************************************/
/*                                                                                                    */
/*                        FUNCIÓN PRINCIPAL                                                           */
/*                                                                                                    */
/******************************************************************************************************/
extern "C" void app_main()
{
    int enconder_auto_last;   // Ultimo valor del Encoder en modo automático
    int enconder_manual_last; // Ultimo valor del encoder en modo manual

    oled_mutex = xSemaphoreCreateMutex(); // Creo oled_mutex para proteger acceso al OLED

    // Inicializaciones
    init_i2c();             // Inicializar I2C
    init_gpio();            // Inicializar GPIO
    init_interrupts(&mode); // Inicializar interrupciones GPIO_1 paso por cero
    timer_init();           // Inicializar el Timer de alta resolución

    init_modbus_uart(); // Inicializar UART para Modbus

    pzem_data_t pzem_data; // Estructura para datos PZEM-004T

    // Crear instancia del display pasando el handle del dispositivo I2C
    oled_display = new SH1106(sh1106_dev_handle);

    // Inicializar el WIFI en modo Estación (STA)
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        ESP_LOGE("MAIN", "Error al inicializar ESP-NOW");
        return;
    }

    // Funcion callback de ESP-NOW cuando recibe datos
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    if (oled_display->probe())
    {
        ESP_LOGI(TAG, "SH1106 128x160 detectado");

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

    // Crear e inicializar el encoder automatico y manual
    rotary_encoder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B, -1000, 1000, 10, 5, 150);
    rotary_encoder->init();

    manual_enconder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B, 0, 1500, 100, 10, 150);
    manual_enconder->init();

    pantalla(); // Mostrar pantalla inicial

    read_pzem_data(&pzem_data); // Leer datos del PZEM-004T

    enconder_auto_last = rotary_encoder->get_count();    // Valor inicial del encoder en modo automático
    enconder_manual_last = manual_enconder->get_count(); // Valor inicial del encoder en modo manual

    update_encoder_display_auto(&pzem_data); // Mostrar valor inicial del encoder en pantalla

    // Crear tarea para leer datos del PZEM-004T periódicamente
    // La he tenido que bajar ya que se bloqueaba el sistema estaba antes de inicializar el oled ya
    // que cuando se crea la tarea empezaba a leer el PZEM-004T, el sistema se bloqueaba al intentar actualizar la pantalla con datos no inicializados
    xTaskCreate(
        leer_pzem_004,   // La funcion que lee el PZEM-004T
        "Leer_PZEM_004", // Nombre de la tarea
        4096,            // Tamaño de la pila
        &pzem_data,      // Parámetro que se le pasa a la tarea
        5,               // Prioridad de la tarea
        NULL             // Handle de la tarea (no se usa)
    );

    while (1)
    {
        estado_botones(); // Leer botones BACK y CONFIRM

        if (mode)
        {
            // Modo AUTOMATICO
            rotary_encoder->update(); // Lee el encoder

            // Actualizar display del encoder si hay cambios
            if (rotary_encoder->get_count() != enconder_auto_last)
            {
                update_encoder_display_auto(&pzem_data);
                enconder_auto_last = rotary_encoder->get_count();
            }
        }
        else
        {
            // Modo MANUAL
            manual_enconder->update(); // Lee el encoder

            // Actualizar display del encoder si hay cambios
            if (manual_enconder->get_count() != enconder_manual_last)
            {
                update_encoder_display_manual();
                enconder_manual_last = manual_enconder->get_count();

                // Pasa de watios a microsegundos
                if (manual_enconder->get_count() > 0)
                {
                    tiempo_espera_us = mapear_potencia_a_retardo((float)enconder_manual_last);
                }
                else
                {
                    tiempo_espera_us = 10000; // Si el valor del encoder manual es 0 no disparo el SRC
                }
            }
        }
        vTaskDelay(1); // Pequeña demora para evitar uso excesivo de CPU
    }
}

/*********************************************************************************************************
                         FUNCIONES AUXILIARES
**********************************************************************************************************/

// Tarea que lee datos del PZEM-004T en paralelo
void leer_pzem_004(void *arg)
{
    pzem_data_t &pzem_data = *(pzem_data_t *)arg; // Tengo que castear el puntero void* al tipo correcto

    // Crear un buffer para guardar el texto
    char buffer[20]; // 20 caracteres son suficientes para "123.4 V"

    esp_err_t ret;

    while (1)
    {
        ret = read_pzem_data(&pzem_data); // Leer datos del PZEM-004T

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error leyendo datos PZEM-004T: %s", esp_err_to_name(ret));
        }

        /*
        ESP_LOGI(TAG, "PZEM-004T Datos: V=%.1fV I=%.3fA P=%.1fW E=%.3fkWh F=%.1fHz PF=%.2f Alarms=0x%04X",
                     pzem_data.voltage,
                     pzem_data.current,
                     pzem_data.power,
                     pzem_data.energy,
                     pzem_data.frequency,
                     pzem_data.power_factor,
                     pzem_data.alarms);
        */

        if (mode) // Modo AUTOMATICO
        {
            // Intenta tomar la llave del mutex para actualizar la pantalla, si no la toma en 50ms
            // no entra, para evitar errores del I2C
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                // Solo actualizo la pantalla si toma el mutex
                snprintf(buffer, sizeof(buffer), "%.0f W", pzem_data.power);
                oled_display->drawString(90, 0, "      "); // Limpiar área anterior
                // Potencia Placa Solar
                oled_display->drawString(90, 0, buffer);
                // Formatear el float dentro del buffer
                snprintf(buffer, sizeof(buffer), "%.0f V", pzem_data.voltage);
                oled_display->drawString(90, 1, "      "); // Limpiar área anterior
                // Voltaje Placa Solar
                oled_display->drawString(90, 1, buffer);

                // itoa utiliza menos memoria que snprintf() 0.0f pero no lo recomiendan
                itoa((rotary_encoder->get_count() + pzem_data.power), buffer, 10);
                if (pzem_data.power < MINIMA_POTENCIA_PLACA)
                {
                    // Si la potencia de la placa es menor que MINIMA_POTENCIA_PLACA, no disparo el SRC
                    oled_display->drawString(70, 3, "  0 W");
                }
                else
                {
                    oled_display->drawString(70, 3, "      ");
                    // Potencia total (placa + encoder + minima potencia)
                    oled_display->drawString(70, 3, buffer);
                }

                snprintf(buffer, sizeof(buffer), "%d", rotary_encoder->get_count());
                oled_display->drawString(70, 5, "     ");
                // Potencia del enconder que le añadira al termo
                oled_display->drawString(70, 5, buffer);
                oled_display->update();
                xSemaphoreGive(oled_mutex); // Devolver la llave
            }

            // Lo meto aqui ya que la potencia de la placa cambia constantemente
            if (pzem_data.power < MINIMA_POTENCIA_PLACA)
            {
                // Si la potencia de la placa es menor que MINIMA_POTENCIA_PLACA, no disparo el SRC
                tiempo_espera_us = 10000; // No disparo el SRC
            }
            else
            {
                // La Potencia del enconder + potencia medida en el PZEM y lo paso a microsegundos
                tiempo_espera_us = mapear_potencia_a_retardo((float)rotary_encoder->get_count() + pzem_data.power);
            }
        }
        else
        {
            // En modo MANUAL no hace falta actualizar nada aquí, la potencia ya la tengo fija
            // La potencia se actualiza en la función principal cuando se mueve el encoder
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Esperar 200 milisegundos antes de la siguiente lectura
    }
}

// Callback del Timer: Genera el pulso de disparo del SRC
void timer_callback(void *arg)
{
    // Si el tiempo de espera es 10000us, significa que no quiero disparar el SRC
    if (tiempo_espera_us == 10000)
    {
        return; // No disparo el SRC
    }

    // Seguridad: Si el tiempo de espera es mayor que 8400us, no disparo el SRC,
    // ya que el módulo SRC no dispara correctamente por encima de ese retardo
    // 8400us son 38W en mis pruebas son 29W, dejo un marjen seguridad
    if (tiempo_espera_us > POTENCIA_SEGURIDAD_SCR)
    {
        // Limitar el tiempo de espera al máximo permitido para el SRC
        tiempo_espera_us = POTENCIA_SEGURIDAD_SCR;
    }

    // 1. Subir el pin (Inicio del pulso)
    gpio_set_level(SCR, 1);

    // Bloqueamos la CPU ANCHO_PULSO_US microsegundos para garantizar el disparo
    // Esperar exactamente ANCHO_PULSO_US microsegundos
    ets_delay_us(ANCHO_PULSO_US);

    // Bajar el pin (Fin del pulso)
    gpio_set_level(SCR, 0);
}

/**
 * @brief Esta es la función que se ejecutará (ISR Handler) para paso por cero
 * @param arg Argumentos para la función
 */
void IRAM_ATTR isr_paso_cero(void *arg)
{
    // Paso de seguridad: Si ya había un timer corriendo (porque si llegaron pulsos muy rápido),
    // lo detenemos para reiniciar la cuenta. Esto hace el sistema "re-disparable".
    esp_timer_stop(timer_handle);

    esp_timer_start_once(timer_handle, tiempo_espera_us); // Iniciar el Timer una sola vez
}

// Leer datos del PZEM-004T
static esp_err_t read_pzem_data(pzem_data_t *data)
{
    uint8_t bytes_to_read = 25; // Leer 25 bytes de datos
    uint8_t response_buffer[bytes_to_read];
    uint16_t response_length = 0;

    // Leer 10 registros (20 bytes)empezando desde 0x0000 (Función 04 - Read Input Registers)
    ESP_ERROR_CHECK(send_modbus_command(MODBUS_SLAVE_ADDR, MODBUS_READ, MODBUS_DIR_READ, 10));

    // Leer respuesta
    esp_err_t ret = read_modbus_response(response_buffer, &response_length, sizeof(response_buffer));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Timeout leyendo respuesta Modbus");
        return ret;
    }

    // Verificar CRC
    if (!verify_crc(response_buffer, response_length))
    {
        ESP_LOGE(TAG, "CRC error en respuesta Modbus");
        return ESP_ERR_INVALID_CRC;
    }

    // Verificar longitud de respuesta
    // response_buffer[2] contiene el byte count de datos
    if (response_length < response_buffer[2] + 5)
    { // 1(addr) + 1(func) + 1(byte count) + (data) + 2(CRC)
        ESP_LOGE(TAG, "Respuesta muy corta: %d bytes", response_length);
        ESP_LOGE(TAG, "response_buffer[2]: %d bytes", response_buffer[2]);
        return ESP_ERR_INVALID_SIZE;
    }

    // Procesar datos
    data->voltage = ((response_buffer[3] << 8) | response_buffer[4]) / 10.0f;
    data->current = ((response_buffer[5] << 8) | response_buffer[6]) / 1000.0f;

    data->power = ((response_buffer[7] << 24) | (response_buffer[8] << 16) | (response_buffer[9] << 8) |
                   response_buffer[10]) /
                  10.0f;

    data->energy = ((response_buffer[11] << 24) | (response_buffer[12] << 16) | (response_buffer[13] << 8) |
                    response_buffer[14]) /
                   1000.0f;

    data->frequency = ((response_buffer[17] << 8) | response_buffer[18]) / 10.0f;
    data->power_factor = ((response_buffer[19] << 8) | response_buffer[20]) / 100.0f;
    data->alarms = (response_buffer[21] << 8) | response_buffer[22];

    return ESP_OK;
}

// Enviar comando Modbus RTU
static esp_err_t send_modbus_command(uint8_t slave_addr, uint8_t function_code,
                                     uint16_t start_addr, uint16_t reg_count)
{
    uint8_t frame[8];

    // Construir trama Modbus
    frame[0] = slave_addr;               // Dirección esclavo
    frame[1] = function_code;            // Código de función leer escribir
    frame[2] = (start_addr >> 8) & 0xFF; // Address high byte
    frame[3] = start_addr & 0xFF;        // Address low byte
    frame[4] = (reg_count >> 8) & 0xFF;  // Register count high byte
    frame[5] = reg_count & 0xFF;         // Register count low byte

    // Calcular CRC
    uint16_t crc = calculate_crc16(frame, 6);
    frame[6] = crc & 0xFF;        // CRC low byte
    frame[7] = (crc >> 8) & 0xFF; // CRC high byte

    // Enviar frame
    int bytes_written = uart_write_bytes(UART_MODBUS_NUM, (const char *)frame, sizeof(frame));
    if (bytes_written != sizeof(frame))
    {
        ESP_LOGE(TAG, "Error enviando comando Modbus");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Función para calcular CRC16 Modbus
static uint16_t calculate_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Inicializar UART para Modbus.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
static void init_modbus_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = MODBUS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = MODBUS_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = 0, // No permitir que el dominio de energía se apague
            .backup_before_sleep = 0,
        },
    };

    ESP_ERROR_CHECK(uart_param_config(UART_MODBUS_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_MODBUS_NUM, MODBUS_TX_PIN, MODBUS_RX_PIN,
                                 MODBUS_RTS_PIN, MODBUS_CTS_PIN));
    ESP_ERROR_CHECK(uart_driver_install(UART_MODBUS_NUM, MODBUS_RX_BUFFER_SIZE,
                                        MODBUS_TX_BUFFER_SIZE, 0, NULL, 0));
}

// Leer respuesta Modbus
static esp_err_t read_modbus_response(uint8_t *buffer, uint16_t *length, uint16_t max_length)
{
    int64_t start_time = esp_timer_get_time();
    uint16_t bytes_read = 0;

    while ((esp_timer_get_time() - start_time) < (MODBUS_RESPONSE_TIMEOUT_MS * 1000))
    {
        int len = uart_read_bytes(UART_MODBUS_NUM, buffer + bytes_read,
                                  max_length - bytes_read, 20 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            bytes_read += len;
            // Pequeña pausa para permitir que lleguen más datos
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (bytes_read >= max_length)
        {
            break;
        }
    }

    *length = bytes_read;
    return (bytes_read > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Verificar CRC de respuesta
static bool verify_crc(const uint8_t *data, uint16_t length)
{
    if (length < 2)
        return false;

    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];
    uint16_t calculated_crc = calculate_crc16(data, length - 2);

    return (received_crc == calculated_crc);
}

/* @brief Función para actualizar la pantalla con los textos sin información.
 * Actualiza pantalla cuanto toma el muttex
 */
void pantalla()
{
    if (oled_display)
    {
        if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
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
            xSemaphoreGive(oled_mutex); // Devolver la llave si la tomó.
        }
    }
}

/**
 * @brief Función para leer el estado de los botones y visualizarlos en la pantalla OLED.
 * Actualiza la pantalla cuando tome el mutex.
 */
void estado_botones()
{
    // Pulsado botón BOTON_PUSH (CAMBIA MODO)
    if (gpio_get_level(BOTON_PUSH) == 0)
    {
        mode = !mode; // Cambiar estado del modo

        if (mode)
        {
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                oled_display->drawString(70, 3, "      "); // Borra potencia manual
                oled_display->drawString(2, 5, "Casa  (W):");
                oled_display->drawString(2, 7, "Modo: AUTOMATICO");
                oled_display->update();
                xSemaphoreGive(oled_mutex); // Devolver la llave.
            }
        }
        else
        {
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                oled_display->drawString(70, 5, "     "); // Borra potencia automática
                oled_display->drawString(2, 7, "Modo: MANUAL     ");
                oled_display->update();
                xSemaphoreGive(oled_mutex); // Devolver la llave si la tomó.
            }
            // Pasa de watios a microsegundos para el modo manual.
            tiempo_espera_us = mapear_potencia_a_retardo((float)manual_enconder->get_count());
            update_encoder_display_manual();
        }

        // Pequeña demora para evitar rebotes
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // pulsado botón CONFIRM
    if (gpio_get_level(BOTON_CONFIRM) == 0)
    {
        mode = !mode; // Cambiar estado del modo

        if (mode)
        {
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                oled_display->drawString(70, 3, "      "); // Borra potencia manual
                oled_display->drawString(2, 5, "Casa  (W):");
                oled_display->drawString(2, 7, "Modo: AUTOMATICO");
                oled_display->update();
                xSemaphoreGive(oled_mutex); // Devolver la llave.
            }
        }
        else
        {
            if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                oled_display->drawString(70, 5, "     "); // Borra potencia automática
                oled_display->drawString(2, 7, "Modo: MANUAL     ");
                oled_display->update();
                xSemaphoreGive(oled_mutex); // Devolver la llave si la tomó.
            }
            // Pasa de watios a microsegundos para el modo manual.
            tiempo_espera_us = mapear_potencia_a_retardo((float)manual_enconder->get_count());
            update_encoder_display_manual();
        }

        // Pequeña demora para evitar rebotes
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Pulsado botón BACK
    if (gpio_get_level(BOTON_BACK) == 0)
    {
        if (!mode)
        {
            manual_enconder->set_count(0);
        }
        else
        {
            rotary_encoder->set_count(0);
        }
    }
}

/**
 * @brief Inicializar I2C.
 * @return true si se inicializa correctamente, false en caso contrario.
 */
bool init_i2c()
{
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

/**
 * @brief Inicializar GPIO.
 * @return true si se inicializan correctamente, false en caso contrario.
 */
bool init_gpio()
{
    ESP_LOGI(TAG, "Inicializando GPIO ...");
    esp_err_t ret;

    // Configurar GPIO  BACK y CONFIRM como entradas
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL << BOTON_BACK) | (1ULL << BOTON_CONFIRM)), // Botones BACK y CONFIRM
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

    // Configurar SCR como salida para el modulo SRC
    io_conf.pin_bit_mask = (1ULL << SCR); // Pin SCR para salida
    io_conf.mode = GPIO_MODE_OUTPUT;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configurando GPIO_2: %s", esp_err_to_name(ret));
        return false;
    }
    gpio_set_level(SCR, 0); // Inicialmente en bajo

    ESP_LOGI(TAG, "GPIO inicializado correctamente");
    return true;
}

/**
 * @brief Inicializa el timer de alta resolución.
 */
void timer_init()
{
    esp_err_t ret;

    // Crear el Timer de Alta Resolución
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,       // Función callback del timer
        .arg = nullptr,                    // No se pasan argumentos
        .dispatch_method = ESP_TIMER_TASK, // Ejecutar en tarea de alta prioridad (más seguro para 530us)
        .name = "disparador_pulso_retardo"};

    ret = esp_timer_create(&timer_args, &timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error creando timer: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Timer de alta resolucion inicializado correctamente");
    }
}

/**
 * @brief Inicializar interrupciones GPIO.
 * @param mode Puntero al modo de operación, automatico true y manual false.
 * @return true si se inicializan correctamente, false en caso contrario.
 */
bool init_interrupts(void *mode)
{
    ESP_LOGI(TAG, "Inicializando interrupciones GPIO ...");
    esp_err_t ret;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PASO_CERO), // Pin GPIO1 para interrupciones
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

    // Instalar el servicio de manejo de interrupciones GPIO_1
    gpio_install_isr_service(0);

    // Asociar la función de manejo de interrupciones al pin GPIO_1
    gpio_isr_handler_add(PASO_CERO, isr_paso_cero, &mode);

    ESP_LOGI(TAG, "IO inicializado correctamente");
    return true;
}

/**
 * @brief Actualizar pantalla del encoder automático.
 *  Esta función se llama cada vez que se detecta un cambio en el encoder automático y actualiza la pantalla con el nuevo valor.
 *  Solo actualiza el área del valor del encoder automático cuando toma el mutex para evitar errores del I2C.
 * @param datos Puntero a la estructura con los datos del PZEM-004T para mostrar en pantalla junto al valor del encoder automático.
 */
void update_encoder_display_auto(pzem_data_t *datos)
{
    if (oled_display && rotary_encoder)
    {

        char buffer[20]; // 20 caracteres son suficientes para "123.4 V"

        if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE) // Intenta tomar la llave
        {
            // Solo actualizo la pantalla si toma el mutex
            snprintf(buffer, sizeof(buffer), "%.0f W", datos->power); // Formatear el float dentro del buffer
            oled_display->drawString(90, 0, "      ");                // Limpiar área anterior
            oled_display->drawString(90, 0, buffer);                  // Potencia Placa Solar

            snprintf(buffer, sizeof(buffer), "%.0f V", datos->voltage);
            oled_display->drawString(90, 1, "      "); // Limpiar área anterior
            oled_display->drawString(90, 1, buffer);   // Pasamos 'buffer' que ahora contiene el texto formateado

            snprintf(buffer, sizeof(buffer), "%d", rotary_encoder->get_count());
            oled_display->drawString(70, 5, "     "); // Limpiar área anterior
            oled_display->drawString(70, 5, buffer);
            oled_display->update();
            xSemaphoreGive(oled_mutex); // Devolver la llave
        }
    }
}

/**
 * @brief Actualizar pantalla del encoder manual.
 *  Esta función se llama cada vez que se detecta un cambio en el encoder manual y actualiza la pantalla con el nuevo valor.
 * Solo actualiza el área del valor del encoder manual cuando toma el mutex para evitar errores del I2C.
 */
void update_encoder_display_manual()
{
    if (oled_display && manual_enconder)
    {
        char buffer[7];

        if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(50)) == pdTRUE) // Intentar tomar la llave
        {
            snprintf(buffer, sizeof(buffer), "%d W", manual_enconder->get_count());
            oled_display->drawString(70, 3, "      "); // Limpiar área anterior
            oled_display->drawString(70, 3, buffer);
            oled_display->update();
            xSemaphoreGive(oled_mutex); // Devolver la llave
        }
    }
}

/**
 * @brief Convierte potencia a tiempo de disparo buscando el primer registro que iguale o supere la potencia.
 * @param potencia Valor de 0 a 1500
 * @return uint64_t Tiempo en microsegundos (0 a 10000)
 */
static uint64_t mapear_potencia_a_retardo(float potencia)
{
    typedef struct
    {
        float potencia;
        uint64_t retardo_us;
    } mapa_t;

    // Usamos 'static const' para que la tabla resida en la memoria Flash y no sature la RAM
    static const mapa_t tabla[60] = {
        {38, 8400}, {55, 8188}, {75, 7980}, {87, 7872}, {100, 7760}, {112, 7674}, {125, 7575}, {136, 7500}, {150, 7410}, {168, 7300}, {175, 7260}, {187, 7190}, {200, 7120}, {215, 7040}, {225, 6990}, {240, 6915}, {250, 6865}, {263, 6805}, {275, 6748}, {288, 6690}, {300, 6637}, {313, 6581}, {325, 6530}, {338, 6475}, {350, 6425}, {362, 6375}, {375, 6325}, {387, 6275}, {400, 6225}, {412, 6178}, {425, 6130}, {437, 6083}, {450, 6035}, {462, 5990}, {475, 5945}, {500, 5853}, {525, 5765}, {550, 5678}, {575, 5590}, {600, 5505}, {625, 5420}, {650, 5335}, {675, 5250}, {700, 5167}, {750, 4999}, {800, 4833}, {850, 4665}, {900, 4495}, {950, 4323}, {1000, 4145}, {1050, 3965}, {1100, 3775}, {1150, 3575}, {1200, 3365}, {1250, 3135}, {1300, 2880}, {1350, 2590}, {1400, 2240}, {1450, 1755}, {1500, 350}};

    // Buscamos en la tabla el valor de potencia más cercano por encima
    for (int i = 0; i < 60; i++)
    {
        if (potencia <= tabla[i].potencia)
        {
            // Debug: Imprimir potencia y retardo para verificar que se mapea correctamente
            // printf("Potencia: %.1f W, Retardo: %llu us\n", tabla[i].potencia, tabla[i].retardo_us);
            return tabla[i].retardo_us;
        }
    }

    return tabla[0].retardo_us; // Valor por defecto si la potencia es menor que el mínimo
}