// RP2040 IMU and Pressure data logger written by Sean Meese
// Started on 08-01-24

#include <iostream>
#include <iomanip>
#include <string>
#include "pico/stdlib.h"
#include "icm_20948.hpp"  // Converted ICM-20948 library
#include "buzzer.hpp"     // Converted buzzer library

int main() {
    stdio_init_all(); // Initialize the I/O for serial communication

    std::cout << "Beginning initialization..." << std::endl;

    // DATA_LED and Buzzer pin initialization
    static constexpr uint DATA_LED = 25;
    gpio_init(DATA_LED);
    gpio_set_dir(DATA_LED, GPIO_OUT);
    Buzzer::initialize();

    // SPI initialization
    ICM20948 IMU;

    sleep_ms(500);

    IMU.reset(); // Using icm_20948 namespace for IMU reset
    sleep_ms(500);

    // Check the device ID and if it doesn't work create an error
    if (IMU.whoAmI() != 0xEA) {
        std::cout << "Device ID is incorrect!" << std::endl;
        while(true);
    }
    sleep_ms(500);

    std::cout << "Board initialization complete..." << std::endl;

    // IMU data structure
    ICM20948::MotionData IMU_data;

    // Main loop
    while (true) {
        gpio_put(DATA_LED, 1); // Turn on data LED using gpio_init

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
                  << ", Z: " << IMU_data.z_mag << "]" << std::endl;

        sleep_ms(500); // Wait 500ms

        gpio_put(DATA_LED, 0); // Turn off data LED using gpio_init
        sleep_ms(500); // Wait 500ms
    }

    return 0;
}
