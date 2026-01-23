#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sh1106.h"
#include "encoder.h"
#include "esp_timer.h"
#include "driver/uart.h"   // Para comunicación Modbus RTU
#include "freertos/task.h" // Para leer PZEM-004T periódicamente

#define TAG "OLED_APP"

// Configuración UART para Modbus RTU
#define UART_MODBUS_NUM UART_NUM_1
#define MODBUS_BAUDRATE 9600
#define MODBUS_RX_PIN GPIO_NUM_20
#define MODBUS_TX_PIN GPIO_NUM_21
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
#define ENCODER_PIN_A GPIO_NUM_7
#define ENCODER_PIN_B GPIO_NUM_10
#define PASO_CERO GPIO_NUM_1

// Configuración I2C
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_FREQ_HZ 100000
#define SH1106_I2C_ADDRESS 0x3C
i2c_master_bus_handle_t bus_handle = nullptr;
i2c_master_dev_handle_t sh1106_dev_handle = nullptr;
SH1106 *oled_display = nullptr; // Instancia global del display

// Configuración Timer y retardo
#include "rom/ets_sys.h"   // Para ets_delay_us
#define ANCHO_PULSO_US 530 // Duración del pulso en ALTO (fijo)
// Variable global para el tiempo de espera antes del disparo (0 a 10.000 us)
volatile uint64_t tiempo_espera_us = 0; // "volatile" es vital para que el compilador sepa que esto cambia en tiempo real
esp_timer_handle_t timer_handle;        // Handle del timer para el retardo

// Modo inicial: true = AUTOMATICO, false = MANUAL
bool mode = true;

// Instancia del encoder
Encoder *rotary_encoder = nullptr;
Encoder *manual_enconder = nullptr;

bool init_i2c();                             // Configura I2C
bool init_gpio();                            // Configura GPIO
bool init_interrupts(void *mode);            // Configura interrupciones GPIO
void IRAM_ATTR isr_paso_cero(void *arg);     // Manejador de interrupciones paso por cero
void pantalla();                             // Funcion imprime en pantalla
void estado_botones();                       // Funcion lee estado botones
void update_encoder_display_auto(void *arg); // FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN MODO AUTOMATICO
void update_encoder_display_manual();        // FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN MODO MANUAL
void verificar_pantalla_128x64();            // Función de diagnóstico de pantalla 128x64
void timer_init();                           // Inicializa el timer de alta resolución
void timer_callback(void *arg);              // Callback del timer de retardo
void leer_pzem_004(void *arg);               // Tarea para leer datos del PZEM-004T

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

static void init_modbus_uart(void);                 // Inicializar UART para Modbus
static esp_err_t read_pzem_data(pzem_data_t *data); // Leer datos del PZEM-004T
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
/*                        FUNCIÓN PRINCIPAL                                                                             */
/*                                                                                                    */
/******************************************************************************************************/
extern "C" void app_main()
{
    int enconder_auto_last;   // Valor del Encoder en modo automático
    int enconder_manual_last; // Valor del encoder en modo manual

    // Inicializaciones
    init_i2c();             // Inicializar I2C
    init_gpio();            // Inicializar GPIO
    init_interrupts(&mode); // Inicializar interrupciones GPIO_1 paso por cero
    timer_init();           // Inicializar el Timer de alta resolución

    init_modbus_uart();    // Inicializar UART para Modbus
    pzem_data_t pzem_data; // Estructura para datos PZEM-004T

    // Crear tarea para leer datos del PZEM-004T periódicamente
    xTaskCreate(
        leer_pzem_004,   // La funcion que lee el PZEM-004T
        "Leer_PZEM_004", // Nombre de la tarea
        4096,            // Tamaño de la pila
        &pzem_data,      // Parámetro que se le pasa a la tarea
        5,               // Prioridad de la tarea
        NULL             // Handle de la tarea (no se usa)
    );

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

    // Crear e inicializar el encoder automatico y manual
    rotary_encoder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B, -1000, 1000, 10, 5, 150);
    rotary_encoder->init();

    manual_enconder = new Encoder(ENCODER_PIN_A, ENCODER_PIN_B, 0, 1500, 100, 10, 150);
    manual_enconder->init();

    pantalla(); // Mostrar pantalla inicial

    read_pzem_data(&pzem_data); // Leer datos del PZEM-004T

    enconder_auto_last = rotary_encoder->get_count();    // Valor inicial del encoder en modo automático
    enconder_manual_last = manual_enconder->get_count(); // Valor inicial del encoder en modo manual
    update_encoder_display_auto(&pzem_data);

    while (1)
    {
        estado_botones(); // Leer botones BACK y CONFIRM

        if (mode)
        { // Modo AUTOMATICO
            rotary_encoder->update();
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
                tiempo_espera_us = 10000.0f - ((float)enconder_manual_last * 6.666667f);
            }
        }
        vTaskDelay(1); // Pequeña demora para evitar uso excesivo de CPU
    }
}

/*********************************************************************************************************
                         FUNCIONES AUXILIARES
**********************************************************************************************************/

// Tarea para leer datos del PZEM-004T
void leer_pzem_004(void *arg)
{
    // Tengo que castear el puntero void* al tipo correcto
    pzem_data_t &pzem_data = *(pzem_data_t *)arg;
    // pzem_data_t pzem_data;
    esp_err_t ret;

    while (1)
    {
        ret = read_pzem_data(&pzem_data);

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
            // 2. Crear un buffer (espacio en memoria) para guardar el texto
            char buffer[20];                                               // 20 caracteres son suficientes para "123.4 V"
            snprintf(buffer, sizeof(buffer), "%.0f V", pzem_data.voltage); // 3. Formatear el float dentro del buffer
            oled_display->drawString(90, 1, buffer);                       // 4. Pasamos 'buffer' que ahora contiene el texto formateado

            snprintf(buffer, sizeof(buffer), "%.0f W", pzem_data.power);
            oled_display->drawString(90, 0, buffer); // Potencia Placa Solar

            oled_display->update();
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar milisegundos antes de la siguiente lectura
    }
}

// Leer datos del PZEM-004T
static esp_err_t read_pzem_data(pzem_data_t *data)
{
    uint8_t bytes_to_read = 25; // Leer 25 bytes de datos
    uint8_t response_buffer[bytes_to_read];
    uint16_t response_length = 0;

    // Leer 10 registros empezando desde 0x0000 (Función 04 - Read Input Registers)
    ESP_ERROR_CHECK(send_modbus_command(MODBUS_SLAVE_ADDR, MODBUS_READ, MODBUS_DIR_READ, 10)); // Leer 10 registro (20 bytes)

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

    // Procesar datos (big-endian)
    data->voltage = (float)((response_buffer[3] << 8) | response_buffer[4]) / 10.0f;
    data->current = (float)((response_buffer[5] << 8) | response_buffer[6]) / 1000.0f;
    data->power = (float)((response_buffer[9] << 8) | response_buffer[10] | (response_buffer[7] << 8) | response_buffer[8]) / 10.0f;
    data->energy = (float)((response_buffer[13] << 8) | response_buffer[14] |
                           (response_buffer[11] << 24) | (response_buffer[12] << 16)) /
                   1000.0f;
    data->frequency = (float)((response_buffer[17] << 8) | response_buffer[18]) / 10.0f;
    data->power_factor = (float)((response_buffer[19] << 8) | response_buffer[20]) / 100.0f;
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
        .source_clk = UART_SCLK_DEFAULT,
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

// Callback del Timer: Genera el pulso de 530us
void timer_callback(void *arg)
{
    if (tiempo_espera_us == 0 || tiempo_espera_us > 9999)
    {
        return; // Si el tiempo de espera es 0 o > 9999 no hacemos nada
    }

    gpio_set_level(GPIO_NUM_2, 1); // 1. Subir el pin (Inicio del pulso)

    // Bloqueamos la CPU 530 microsegundos para garantizar precisión absoluta
    ets_delay_us(ANCHO_PULSO_US); // 2. Esperar exactamente 530 microsegundos

    gpio_set_level(GPIO_NUM_2, 0); // 3. Bajar el pin (Fin del pulso)
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

void estado_botones()
{
    // pulsado botón CONFIRM (GPIO6)
    if (gpio_get_level(GPIO_NUM_6) == 0)
    {
        mode = !mode; // Cambiar estado del modo

        if (mode)
        {
            oled_display->drawString(70, 3, "     "); // Borra potencia manual
            oled_display->drawString(2, 5, "Casa  (W):");
            oled_display->drawString(2, 7, "Modo: AUTOMATICO");
            oled_display->update();
        }
        else
        {
            oled_display->drawString(70, 5, "     "); // Borra potencia automática
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

// FUNCIÓN PARA INICIALIZAR I2C
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

//  FUNCIÓN PARA INICIALIZAR GPIO
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

// Inicializa el timer de alta resolución
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

// Esta es la función que se ejecutará (ISR Handler) para paso por cero
void IRAM_ATTR isr_paso_cero(void *arg)
{
    // Paso de seguridad: Si ya había un timer corriendo (porque llegaron pulsos muy rápido),
    // lo detenemos para reiniciar la cuenta. Esto hace el sistema "re-disparable".
    esp_timer_stop(timer_handle);

    esp_timer_start_once(timer_handle, tiempo_espera_us); // Iniciar el Timer una sola vez
}

// Funcion para incializar el GPIO con interrupcion
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

//  FUNCIÓN PARA MOSTRAR VALOR DEL ENCODER EN PANTALLA EN MODO AUTOMATICO
void update_encoder_display_auto(void *arg)
{
    if (oled_display && rotary_encoder)
    {
        // 1. Castear (convertir) el argumento void* a tu estructura
        pzem_data_t *datos = (pzem_data_t *)arg;

        if (datos == NULL)
            return; // Verificar que los datos no sean nulos para evitar reinicios

        // 2. Crear un buffer (espacio en memoria) para guardar el texto
        char buffer[18]; // 20 caracteres son suficientes para "123.4 V"

        snprintf(buffer, sizeof(buffer), "%.0f V", datos->voltage); // 3. Formatear el float dentro del buffer

        oled_display->drawString(90, 1, buffer); // 4. Pasamos 'buffer' que ahora contiene el texto formateado

        snprintf(buffer, sizeof(buffer), "%.0f W", datos->power);
        oled_display->drawString(90, 0, buffer); // Potencia Placa Solar

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
        // itoa utiliza menos memoria que snprintf()
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