#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include "esp_log.h"

class Encoder {
private:
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    
    // Variables de estado
    int count;
    uint8_t last_state;
    uint32_t last_time;
    
    // Configuración
    int min_value;
    int max_value;
    int fast_increment;
    int slow_increment;
    uint32_t fast_threshold_ms;
    
    // Métodos privados
    uint8_t read_state();
    int calculate_direction(uint8_t current_state, uint8_t last_state);

public:
    // Constructor
    Encoder(gpio_num_t pin_a, gpio_num_t pin_b, 
            int min_val = -1000, int max_val = 1000,
            int fast_inc = 10, int slow_inc = 1,
            uint32_t fast_threshold = 150);
    
    
    // Inicialización
    bool init();
    
    // Métodos principales
    void update();
    int get_count() const;
    void set_count(int value);
    void reset();
    
    // Configuración
    void set_limits(int min_val, int max_val);
    void set_increments(int fast_inc, int slow_inc);
    void set_fast_threshold(uint32_t threshold_ms);
};

#endif // ENCODER_H