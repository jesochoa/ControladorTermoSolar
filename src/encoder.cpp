#include "encoder.h"
#include "freertos/FreeRTOS.h"


Encoder::Encoder(gpio_num_t pin_a, gpio_num_t pin_b, 
                 int min_val, int max_val,
                 int fast_inc, int slow_inc,
                 uint32_t fast_threshold)
    : pin_a(pin_a), pin_b(pin_b), 
      count(0), last_state(0), last_time(0),
      min_value(min_val), max_value(max_val),
      fast_increment(fast_inc), slow_increment(slow_inc),
      fast_threshold_ms(fast_threshold) {
}

bool Encoder::init() {
    // Configurar pines GPIO
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin_a) | (1ULL << pin_b);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Error configurando GPIO: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Leer estado inicial
    last_state = read_state();
    last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;  // Usar FreeRTOS en lugar de esp_timer
    
    ESP_LOGI("ENCODER", "Encoder inicializado. Pines: A=%d, B=%d", pin_a, pin_b);
    
    return true;
}

uint8_t Encoder::read_state() {
    uint8_t encoder_a = gpio_get_level(pin_a);
    uint8_t encoder_b = gpio_get_level(pin_b);
    return (encoder_a << 1) | encoder_b;
}

int Encoder::calculate_direction(uint8_t current_state, uint8_t last_state) {
    static const int8_t transitions[] = {
        0,  1, -1, 0,    // De 00 a: 00, 01, 10, 11
        -1, 0,  0, 1,    // De 01 a: 00, 01, 10, 11  
        1,  0,  0, -1,   // De 10 a: 00, 01, 10, 11
        0, -1,  1, 0     // De 11 a: 00, 01, 10, 11
    };
    
    uint8_t transition_index = (last_state << 2) | current_state;
    return transitions[transition_index];
}

void Encoder::update() {
    uint8_t current_state = read_state();
    
    if (current_state != last_state) {
        int direction = calculate_direction(current_state, last_state);
        
        static uint8_t change_count = 0;
        change_count++;
        
        if (direction != 0 && (change_count % 2 == 0)) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;  // FreeRTOS
            uint32_t time_since_last = (last_time > 0) ? current_time - last_time : 0;
            
            int increment = slow_increment;
            
            if (time_since_last > 0 && time_since_last < fast_threshold_ms) {
                increment = fast_increment;
                //ESP_LOGI("ENCODER", " GIRO RAPIDO! Tiempo: %lu ms", time_since_last);
            }
            
            int new_count = count + (direction * increment);
            
            // Limitar al rango
            if (new_count > max_value) new_count = max_value;
            if (new_count < min_value) new_count = min_value;
            
            count = new_count;
            last_time = current_time;
            
            //ESP_LOGI("ENCODER", "Encoder: %d", count);
        }
        
        last_state = current_state;
    }
}

int Encoder::get_count() const {
    return count;
}

void Encoder::set_count(int value) {
    if (value >= min_value && value <= max_value) {
        count = value;
    }
}

void Encoder::reset() {
    count = 0;
    last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void Encoder::set_limits(int min_val, int max_val) {
    min_value = min_val;
    max_value = max_val;
    if (count < min_value) count = min_value;
    if (count > max_value) count = max_value;
}

void Encoder::set_increments(int fast_inc, int slow_inc) {
    fast_increment = fast_inc;
    slow_increment = slow_inc;
}

void Encoder::set_fast_threshold(uint32_t threshold_ms) {
    fast_threshold_ms = threshold_ms;
}