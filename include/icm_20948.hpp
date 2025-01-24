#ifndef ICM_20948_HPP
#define ICM_20948_HPP

#include <cstdint>
#include <iostream> // For std::cout
#include <iomanip>  // For formatted output
#include "pico/stdlib.h"
#include "hardware/spi.h"

/**
 * @brief Class to interface with the ICM-20948 9-axis IMU sensor.
 */
class ICM20948 {
public:
    // Register definitions
    static constexpr uint8_t IMU_DEV_ID     = 0x00; // WHO AM I device ID register [7:0], should return 0xEA
    static constexpr uint8_t USER_CTRL      = 0x03; // Digital Motion processor enable, FIFO_EN, I2C_MST_EN, I2C_IF_DIS, DMP_RST, SRAM_RST, I2C_MST_RST, blank
    static constexpr uint8_t LP_CONFIG      = 0x05; // Blank, i2C_MST_CYCLE, ACCEL_CYCLE, GYRO_CYCLE, blank[3:0]
    static constexpr uint8_t PWR_MGMT_1     = 0x06; // Device reset, sleep, LP_EN, blank, TEMP_DIS, CLKSEL[2:0]
    static constexpr uint8_t PWR_MGMT_2     = 0x07; // Blank [7:6], DISABLE_ACCEL [5:3], DISABLE_GYRO [2:0]
    static constexpr uint8_t REG_BANK_SEL   = 0x7F; // Blank [7:6], USER_BANK (0-3) [5:4], blank [3:0]
    static constexpr uint8_t ODR_ALIGN_EN   = 0x09; // Align sensor readings to time register, blank [7:1], 1 to enable time alignment [0]

    // Accelerometer configuration registers
    static constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0X10;
    static constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0X10;
    static constexpr uint8_t ACCEL_CONFIG       = 0X14;

    // Accelerometer data registers (acceleration = ACCEL_XOUT/Accel_Sensitivity)
    static constexpr uint8_t ACCEL_XOUT_H   = 0x2D; // X axis acceleration high byte
    static constexpr uint8_t ACCEL_XOUT_L   = 0x2E; // X axis acceleration low byte
    static constexpr uint8_t ACCEL_YOUT_H   = 0x2F; // Y axis acceleration high byte
    static constexpr uint8_t ACCEL_YOUT_L   = 0x30; // Y axis acceleration low byte
    static constexpr uint8_t ACCEL_ZOUT_H   = 0x31; // Z axis acceleration high byte
    static constexpr uint8_t ACCEL_ZOUT_L   = 0x32; // Z axis acceleration low byte

    // Gyro configuration registers
    static constexpr uint8_t GYRO_SMPLRT_DIV    = 0x00;
    static constexpr uint8_t GYRO_CONFIG_1      = 0x01;
    static constexpr uint8_t GYRO_CONFIG_2      = 0x02;
    static constexpr uint8_t XG_OFFS_USRH       = 0x03;
    static constexpr uint8_t XG_OFFS_USRL       = 0x04;
    static constexpr uint8_t YG_OFFS_USRH       = 0x05;
    static constexpr uint8_t YG_OFFS_USRL       = 0x06;
    static constexpr uint8_t ZG_OFFS_USRH       = 0x07;
    static constexpr uint8_t ZG_OFFS_USRL       = 0x08;

    // Gyro data registers (angular rate = GYRO_XOUT/Gyro_Sensitivity)
    static constexpr uint8_t GYRO_XOUT_H    = 0x33;    // X axis gyro high byte
    static constexpr uint8_t GYRO_XOUT_L    = 0x34;    // X axis gyro low byte
    static constexpr uint8_t GYRO_YOUT_H    = 0x35;    // Y axis gyro high byte
    static constexpr uint8_t GYRO_YOUT_L    = 0x36;    // Y axis gyro low byte
    static constexpr uint8_t GYRO_ZOUT_H    = 0X37;    // Z axis gyro high byte
    static constexpr uint8_t GYRO_ZOUT_L    = 0x38;    // Z axis gyro low byte

    // Temperature data registers
    static constexpr uint8_t IMU_TEMP_H     = 0x39; // IMU temperature high byte
    static constexpr uint8_t IMU_TEMP_L     = 0x3A; // IMU temperature low byte

    // I2C Master Control registers
    static constexpr uint8_t I2C_MST_CTRL           = 0x01;    // Enable I2C Master to enable magnetometer data
    static constexpr uint8_t I2C_MST_ODR_CONFIG     = 0x00;    // Set data rate

    // AK09916 magnetometer registers
    static constexpr uint8_t I2C_SLV0_ADDR   = 0x03;
    static constexpr uint8_t I2C_SLV0_REG    = 0x04;
    static constexpr uint8_t I2C_SLV0_CTRL   = 0x05;
    static constexpr uint8_t I2C_SLV0_D0     = 0x06;
    static constexpr uint8_t AK09916_ADDRESS = 0x0c;
    static constexpr uint8_t MAG_CTRL2       = 0x31;
    static constexpr uint8_t MAG_CTRL3       = 0x32;
    static constexpr uint8_t MAG_DATA_ONSET  = 0x11;

// ***** USER CONFIGURATION START *****
/*
    Sensitivity range for gyroscope and accelerometer:
    00 = 250dps  or 2g  - 0x00
    01 = 500dps  or 4g  - 0x01
    10 = 1000dps or 8g  - 0x02
    11 = 2000dps or 16g - 0x03

    Gyroscope sample rate divider values, sample rate in Hz, and 
    associated # of samples to average 
    (highest average samples shown, lesser values are still valid):
    0   - 1.1  kHz disabled - 0x00
    1   - 562.5 Hz - 1x     - 0x01
    2   - 375.0 Hz - 2x     - 0x01
    3   - 281.3 Hz - 4x     - 0x03
    4   - 225.0 Hz - 4x     - 0x04
    5   - 187.5 Hz - 8x     - 0x05
    7   - 140.6 Hz - 8x     - 0x07
    8   - 125.0 Hz - 8x     - 0x08
    10  - 102.3 Hz - 16x    - 0x0A
    15  - 70.3  Hz - 16x    - 0x0F
    16  - 66.2  Hz - 16x    - 0x10
    22  - 48.9  Hz - 32x    - 0x16
    31  - 35.2  Hz - 32x    - 0x1F
    32  - 34.1  Hz - 32x    - 0x20
    63  - 17.6  Hz - 64x    - 0x3F
    64  - 17.3  Hz - 64x    - 0x40
    255 - 4.4   Hz - 128x   - 0xFF

    Averaging filter configuration settings and associated bit value:
    0 - 1x   - 0x00
    1 - 2x   - 0x01
    2 - 4x   - 0x02
    3 - 8x   - 0x03
    4 - 16x  - 0x04
    5 - 32x  - 0x05
    6 - 64x  - 0x06
    7 - 128x - 0x07
*/
    static constexpr uint8_t GYRO_SENSITIVITY  = 0x01;      // Set the gyroscope sensitivity range
    static constexpr uint8_t GYRO_SAMPLE_RATE  = 0x00;      // Set the sample rate divider value
    static constexpr uint8_t GYRO_AVERAGING    = 0x00;      // Set the # of samples to average

    static constexpr uint8_t ACCEL_RANGE_VALUE = 0x0a;      // Set the accelerometer sensitivity range
    static constexpr uint8_t ACCEL_SAMPLE_RATE = 0x00;      // Set the accelerometer sample rate

    static constexpr uint8_t ODR               = 0x03;      // Set the output data rate through I2C (init_magnetometer func)
    static constexpr auto BIAS_SAMPLES         = 100;       // Set the # of samples for bias removal

    // Convert bias sample rates
    static constexpr auto BIAS_RATE          = (4 * BIAS_SAMPLES);
    static constexpr auto MAG_BIAS_SAMPLES   = (10 * BIAS_SAMPLES);

    // Conversions for accelerometer MSB and LSB configuration registers
    static constexpr uint8_t ACCEL_SAMPLE_RATE_MSB = ((ACCEL_SAMPLE_RATE & 0xF0) >> 4); // Get most significant byte of sample rate
    static constexpr uint8_t ACCEL_SAMPLE_RATE_LSB = (ACCEL_SAMPLE_RATE & 0x0F);        // Get least significant byte of sample rate

    /**
     * @brief Motion data structure to hold IMU readings.
     */
    struct MotionData {
        int16_t x_accel;
        int16_t y_accel;
        int16_t z_accel;
        int16_t x_gyro;
        int16_t y_gyro;
        int16_t z_gyro;
        int16_t x_mag;
        int16_t y_mag;
        int16_t z_mag;
    };

    enum class AccelRange {
        G_2,
        G_4,
        G_8,
        G_16
    };

    enum class GyroRange {
        DPS_250,
        DPS_500,
        DPS_1000,
        DPS_2000
    };

    enum class UserBank {
        Bank0 = 0,
        Bank1 = 1 << 4,
        Bank2 = 2 << 4,
        Bank3 = 3 << 4
    };

    // Constructor
    ICM20948(int CS, int MISO, int MOSI, int SCLK, spi_inst_t* SPI);

    // Basic control functions
    void reset();
    uint8_t whoAmI();
    void selectBank(UserBank bank);
    void writeRegister(UserBank bank, uint8_t reg, uint8_t data);
    uint8_t readRegister(UserBank bank, uint8_t reg);

    // Magnetometer-specific functions
    void initMagnetometer();
    void writeMagRegister(uint8_t reg, uint8_t data);
    void readMagRegister(uint8_t onsetReg, uint8_t len);

    // IMU data functions
    MotionData readMotionData();
    void removeGyroBias();
    void calibrateMagnetometer();

private:
    int IMU_CS_PIN;
    spi_inst_t* SPI_PORT;
};

#endif // ICM_20948_HPP
