#include "pzem_modbus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PZEM_MODBUS";

PzemModbus::PzemModbus(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, uint8_t slave_addr)
    : _uart_num(uart_num), _tx_pin(tx_pin), _rx_pin(rx_pin), _slave_addr(slave_addr) {}

PzemModbus::~PzemModbus() {
    if (uart_is_driver_installed(_uart_num)) {
        uart_driver_delete(_uart_num);
    }
}

esp_err_t PzemModbus::init() {
    uart_config_t uart_config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = { .allow_pd = 0 }
    };

    esp_err_t ret = uart_param_config(_uart_num, &uart_config);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(_uart_num, _tx_pin, _rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    return uart_driver_install(_uart_num, RX_BUFFER_SIZE, TX_BUFFER_SIZE, 0, NULL, 0);
}

esp_err_t PzemModbus::send_command(uint8_t function_code, uint16_t start_addr, uint16_t reg_count) {
    uint8_t frame[8];

    frame[0] = _slave_addr;
    frame[1] = function_code;
    frame[2] = (start_addr >> 8) & 0xFF;
    frame[3] = start_addr & 0xFF;
    frame[4] = (reg_count >> 8) & 0xFF;
    frame[5] = reg_count & 0xFF;

    uint16_t crc = calculate_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    int bytes_written = uart_write_bytes(_uart_num, (const char *)frame, sizeof(frame));
    return (bytes_written == sizeof(frame)) ? ESP_OK : ESP_FAIL;
}

esp_err_t PzemModbus::read_response(uint8_t *buffer, uint16_t *length, uint16_t max_length) {
    int64_t start_time = esp_timer_get_time();
    uint16_t bytes_read = 0;

    while ((esp_timer_get_time() - start_time) < (RESPONSE_TIMEOUT_MS * 1000)) {
        int len = uart_read_bytes(_uart_num, buffer + bytes_read, max_length - bytes_read, pdMS_TO_TICKS(20));

        if (len > 0) {
            bytes_read += len;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (bytes_read >= max_length) {
            break;
        }
    }

    *length = bytes_read;
    return (bytes_read > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t PzemModbus::read_data(pzem_data_t *data) {
    uint8_t bytes_to_read = 25; 
    uint8_t response_buffer[bytes_to_read];
    uint16_t response_length = 0;

    esp_err_t ret = send_command(CMD_READ_INPUT_REG, START_REG_ADDR, 10);
    if (ret != ESP_OK) return ret;

    ret = read_response(response_buffer, &response_length, sizeof(response_buffer));
    if (ret != ESP_OK) return ret;

    if (!verify_crc(response_buffer, response_length)) {
        return ESP_ERR_INVALID_CRC;
    }

    if (response_length < response_buffer[2] + 5) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Parseo de registros Modbus
    data->voltage      = ((response_buffer[3] << 8)  | response_buffer[4]) / 10.0f;
    data->current      = ((response_buffer[5] << 8)  | response_buffer[6]) / 1000.0f;
    data->power        = ((response_buffer[7] << 24) | (response_buffer[8] << 16) | (response_buffer[9] << 8) | response_buffer[10]) / 10.0f;
    data->energy       = ((response_buffer[11] << 24)| (response_buffer[12] << 16)| (response_buffer[13] << 8) | response_buffer[14]) / 1000.0f;
    data->frequency    = ((response_buffer[17] << 8) | response_buffer[18]) / 10.0f;
    data->power_factor = ((response_buffer[19] << 8) | response_buffer[20]) / 100.0f;
    data->alarms       = (response_buffer[21] << 8)  | response_buffer[22];

    return ESP_OK;
}

uint16_t PzemModbus::calculate_crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

bool PzemModbus::verify_crc(const uint8_t *data, uint16_t length) {
    if (length < 2) return false;
    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];
    uint16_t calculated_crc = calculate_crc16(data, length - 2);
    return (received_crc == calculated_crc);
}