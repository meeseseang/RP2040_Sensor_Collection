#ifndef BUZZER_HPP
#define BUZZER_HPP

#include "pico/stdlib.h"

/**
 * @brief Buzzer control class for the Pico microcontroller.
 */
class Buzzer {
public:
    /// Pin used for the buzzer.
    static constexpr uint BUZZER_PIN = 12;

    /**
     * @brief Initializes the buzzer pin.
     */
    static void initialize();

    /**
     * @brief Makes a long beep lasting 1 second.
     */
    static void long_beep();

    /**
     * @brief Makes multiple beeps.
     * @param beeps Number of beeps to emit.
     * @param delay Delay in milliseconds between beeps.
     */
    static void multi_beep(unsigned int beeps, unsigned int delay);
};

#endif // BUZZER_HPP
