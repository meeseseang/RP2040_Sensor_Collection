#include "gpio_init.hpp"

namespace GPIO {

    void initSPI() {
        // SPI0 interface initialization
        spi_init(SPI_PORT, 1000000); // Initialize spi0 at 1MHz
        gpio_set_function(MISO, GPIO_FUNC_SPI);
        gpio_set_function(SCLK, GPIO_FUNC_SPI);
        gpio_set_function(MOSI, GPIO_FUNC_SPI);

        // SPI1 interface initialization
        spi_init(SPI1_PORT, 1000000); // Initialize spi1 at 1MHz
        gpio_set_function(MISO1, GPIO_FUNC_SPI);
        gpio_set_function(MOSI1, GPIO_FUNC_SPI);
        gpio_set_function(SCLK1, GPIO_FUNC_SPI);

        // Configure chip select for IMU and Pressure sensor
        // Drive CS pin high because it is active low
        gpio_init(CS_IMU);
        gpio_set_dir(CS_IMU, GPIO_OUT);
        gpio_put(CS_IMU, 1);

        gpio_init(CS_PRESS);
        gpio_set_dir(CS_PRESS, GPIO_OUT);
        gpio_put(CS_PRESS, 1);
    }

    void initGPIO() {
        // Configure output LED
        gpio_init(DATA_LED);
        gpio_set_dir(DATA_LED, GPIO_OUT);

        // Configure buzzer
        gpio_init(BUZZER);
        gpio_set_dir(BUZZER, GPIO_OUT);
    }

} // namespace GPIO
