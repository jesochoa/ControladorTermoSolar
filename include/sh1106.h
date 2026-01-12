#ifndef SH1106_H
#define SH1106_H

#include "driver/i2c_master.h"
#include <string.h>

class SH1106 {
public:
    SH1106(i2c_master_dev_handle_t dev_handle);
    ~SH1106();
    
    // Inicialización
    bool init();
    bool probe();

    // Control básico
    void clear();
    void update();

    // Dibujo
    void drawChar(uint8_t x, uint8_t y, char c);
    void drawString(uint8_t x, uint8_t y, const char *str);
    void setPosition(uint8_t page, uint8_t column);
    void drawPixel(uint8_t x, uint8_t y, bool state = true);
    
    // Constantes públicas
    static const uint8_t WIDTH = 130;  // Ancho en píxeles 
    static const uint8_t HEIGHT = 64;  // Altura en píxeles
    static const uint8_t NUM_PAGES = HEIGHT / 8;

private:
    bool writeCommand(uint8_t cmd);
    bool writeData(const uint8_t *data, size_t len);
    
    i2c_master_dev_handle_t device_handle;
    uint8_t oled_buffer[WIDTH * NUM_PAGES];
    
    // Comandos SH1106
    static const uint8_t SET_CONTRAST = 0x81;
    static const uint8_t SET_SEG_REMAP = 0xA0;
    static const uint8_t SET_ENTIRE_ON = 0xA4;
    static const uint8_t SET_NORM_INV = 0xA6;
    static const uint8_t SET_DISP = 0xAE;
    static const uint8_t SET_MEM_ADDR_MODE = 0x20;
    static const uint8_t SET_COL_ADDR = 0x21;
    static const uint8_t SET_PAGE_ADDR = 0x22;
    static const uint8_t SET_DISP_START_LINE = 0x40;
    static const uint8_t SET_SCAN_DIR = 0xC0;
    static const uint8_t SET_COM_PIN_CFG = 0xDA;
    static const uint8_t SET_DISP_OFFSET = 0xD3;
    static const uint8_t SET_CLK_DIV = 0xD5;
    static const uint8_t SET_PRECHARGE = 0xD9;
    static const uint8_t SET_VCOM_DESEL = 0xDB;
};

#endif