// RP2040 IMU and Pressure data logger written by Sean Meese
// Converted to C++17 by [Your Name]
// Started on 08-01-24

#include <iostream>
#include <iomanip>
#include <string>
#include "pico/stdlib.h"
#include "gpio_init.hpp"  // Converted gpio_init library
#include "icm_20948.hpp"  // Converted ICM-20948 library
#include "buzzer.hpp"     // Converted buzzer library

int main() {
    stdio_init_all(); // Initialize the I/O for serial communication

    // Initialization
    std::cout << "Beginning initialization..." << std::endl;
    sleep_ms(1000);

    // Initialize SPI, GPIO, IMU
    GPIO::initSPI();  // Using gpio_init namespace for SPI
    sleep_ms(1000);

    GPIO::initGPIO(); // Using gpio_init namespace for GPIO
    sleep_ms(1000);

    ICM20948::reset(); // Using icm_20948 namespace for IMU reset
    sleep_ms(1000);

    ICM20948::whoAmI(); // Using icm_20948 namespace for IMU identification
    sleep_ms(1000);

    std::cout << "Board initialization complete..." << std::endl;

    // IMU data structure
    ICM20948::MotionData IMU_data;

    // Main loop
    while (true) {
        gpio_put(GPIO::DATA_LED, 1); // Turn on data LED using gpio_init

        ICM20948::readMotionData(IMU_data); // Reading IMU data from icm_20948 namespace

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
                  << ", Z: " << IMU_data.z_mag << "]" << std::endl;

        sleep_ms(500); // Wait 500ms

        gpio_put(GPIO::DATA_LED, 0); // Turn off data LED using gpio_init
        sleep_ms(500); // Wait 500ms
    }

    return 0;
}
