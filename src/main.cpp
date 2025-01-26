// RP2040 IMU and Pressure data logger written by Sean Meese
// Started on 08-01-24

#include <iostream>
#include <iomanip>
#include <string>
#include "pico/stdlib.h"
#include "icm_20948.hpp"  // Converted ICM-20948 library
#include "dps_310.hpp"

// Setup SPI Variables here
#define MISO 16
#define MOSI 19
#define SCLK 18
#define SPI_BUS spi0
#define SPI_SPEED 1 // in MHz

// CS Pins
#define IMU_CS 17
#define PRESS_CS 21

int main() {
    stdio_init_all(); // Initialize the I/O for serial communication

    sleep_ms(10000);
    std::cout << "Beginning initialization..." << std::endl;

    // SPI0 interface initialization
    spi_init(SPI_BUS, (SPI_SPEED*(10e6))); // Initialize spi0 at 1MHz
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    spi_set_format( SPI_BUS,           // SPI instance
                    8,              //Number of bits per transfer
                    SPI_CPOL_1,     // Polarity (CPOL)
                    SPI_CPHA_1,     // Phase (CPHA)
                    SPI_MSB_FIRST);

    // DATA_LED and Buzzer pin initialization
    static constexpr uint DATA_LED = 12;
    gpio_init(DATA_LED);
    gpio_set_dir(DATA_LED, GPIO_OUT);

    // DPS310 init
    DPS310 pressSense(PRESS_CS, SPI_BUS);
    sleep_ms(50);
    pressSense.config(0x80,0x07,0x23,0x23);

    std::cout << "Board initialization complete..." << std::endl;
    sleep_ms(2500);

    // IMU data structure
    //ICM20948::MotionData IMU_data;

    // Main loop
    while (true) {
        gpio_put(DATA_LED, 1);

        /*
        ICM20948::MotionData IMU_data = IMU.readMotionData(); // Reading IMU data from icm_20948 namespace

        // Print IMU data
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Accelerometer: [X: " << IMU_data.x_accel
                  << ", Y: " << IMU_data.y_accel
                  << ", Z: " << IMU_data.z_accel << "]\n";
        std::cout << "Gyroscope: [X: " << IMU_data.x_gyro
                  << ", Y: " << IMU_data.y_gyro
                  << ", Z: " << IMU_data.z_gyro << "]\n";
        std::cout << "Magnetometer: [X: " << IMU_data.x_mag
                  << ", Y: " << IMU_data.y_mag
                  << ", Z: " << IMU_data.z_mag << "]\n\n\n" << std::endl;
        */
        DPS310::PressureData scaledDat = pressSense.scaledData(true, true);
        float scaled_pressure = scaledDat.pressure;
        float scaled_temp = scaledDat.temperature;
        std::cout << "Pressure: " << scaled_pressure << " psi" << std::endl;
        std::cout << "Temperature: " << scaled_temp << " F\n" << std::endl;

        gpio_put(DATA_LED, 0);
        sleep_ms(1500);
    }

    return 0;
}
