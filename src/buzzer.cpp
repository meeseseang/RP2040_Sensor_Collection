#include "buzzer.hpp"

void Buzzer::initialize() {
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

void Buzzer::long_beep() {
    gpio_put(BUZZER_PIN, 1);
    sleep_ms(1000);
    gpio_put(BUZZER_PIN, 0);
}

void Buzzer::multi_beep(unsigned int beeps, unsigned int delay) {
    for (unsigned int i = 0; i < beeps; ++i) {
        gpio_put(BUZZER_PIN, 1);
        sleep_ms(delay);
        gpio_put(BUZZER_PIN, 0);
        sleep_ms(delay); // Optional: Add rest delay between beeps
    }
}
