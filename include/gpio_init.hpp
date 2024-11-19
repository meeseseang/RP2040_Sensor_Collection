#ifndef GPIO_INIT_HPP
#define GPIO_INIT_HPP

#include "pico/stdlib.h"
#include "hardware/spi.h"

namespace GPIO {

    // SPI0 ports
    constexpr spi_inst_t* SPI_PORT = spi0;
    constexpr uint MISO = 16;   // Master In Slave Out
    constexpr uint MOSI = 19;   // Master Out Slave In
    constexpr uint SCLK = 18;   // SPI Clock
    constexpr uint CS_IMU = 17; // Chip select for IMU
    constexpr uint CS_PRESS = 5; // Chip select for pressure sensor

    // SPI1 ports
    constexpr spi_inst_t* SPI1_PORT = spi1;
    constexpr uint MISO1 = 28;
    constexpr uint MOSI1 = 27;
    constexpr uint SCLK1 = 26;
    constexpr uint CS_MICROSD = 29;

    // Status LED pins
    constexpr uint DATA_LED = 25;

    // Buzzer pin
    constexpr uint BUZZER = 12;

    // Function prototypes
    void initSPI();
    void initGPIO();

} // namespace GPIO

#endif // GPIO_INIT_HPP
