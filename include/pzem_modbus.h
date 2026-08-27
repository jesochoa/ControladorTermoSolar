#ifndef PZEM_MODBUS_H
#define PZEM_MODBUS_H

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Estructura para almacenar los datos del PZEM-004T
typedef struct {
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float power_factor;
    uint16_t alarms;
} pzem_data_t;

class PzemModbus {
public:
    // Constructor con los parámetros de configuración por defecto
    PzemModbus(uart_port_t uart_num = UART_NUM_1, 
               gpio_num_t tx_pin = GPIO_NUM_3, 
               gpio_num_t rx_pin = GPIO_NUM_4, 
               uint8_t slave_addr = 0x01);

    ~PzemModbus();

    // Inicializa el periférico UART con el protocolo Modbus RTU
    esp_err_t init();

    // Lee los registros del PZEM-004T y actualiza la estructura de datos
    esp_err_t read_data(pzem_data_t *data);

private:
    uart_port_t _uart_num;
    gpio_num_t _tx_pin;
    gpio_num_t _rx_pin;
    uint8_t _slave_addr;

    // Métodos privados internos de Modbus
    esp_err_t send_command(uint8_t function_code, uint16_t start_addr, uint16_t reg_count);
    esp_err_t read_response(uint8_t *buffer, uint16_t *length, uint16_t max_length);
    uint16_t calculate_crc16(const uint8_t *data, uint16_t length);
    bool verify_crc(const uint8_t *data, uint16_t length);

    // Constantes de configuración Modbus fijas para el PZEM
    static const uint32_t BAUDRATE = 9600;
    static const uint32_t RESPONSE_TIMEOUT_MS = 200;
    static const uint16_t RX_BUFFER_SIZE = 256;
    static const uint16_t TX_BUFFER_SIZE = 256;
    static const uint8_t  CMD_READ_INPUT_REG = 0x04;    
    static const uint16_t START_REG_ADDR = 0x0000;  // Dirección inicial para leer registros de entrada
};

#endif // PZEM_MODBUS_H