#include "icm_20948.hpp"

// Constructor
ICM20948::ICM20948()
{
    // Initialization routines
    gpio_init(IMU_CS_PIN);
    gpio_set_dir(IMU_CS_PIN, GPIO_OUT);
    gpio_put(IMU_CS_PIN, 1); // Set CS high initially
}

// Read WHO_AM_I register
uint8_t ICM20948::whoAmI()
{
    uint8_t reg = IMU_DEV_ID | 0x80;
    uint8_t data;
    gpio_put(IMU_CS_PIN, 0);
    sleep_ms(5);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(IMU_CS_PIN, 1);

    // Use std::cout for output
    std::cout << "WHO_AM_I register: 0x" << std::hex << std::uppercase 
              << static_cast<int>(data) << std::dec << std::endl;
    return data;
}

// Select user bank
void ICM20948::selectBank(UserBank bank)
{
    gpio_put(IMU_CS_PIN, 0);
    sleep_ms(5);
    uint8_t addr = REG_BANK_SEL;
    spi_write_blocking(SPI_PORT, &addr, 1);
    uint8_t bank_value = static_cast<uint8_t>(bank);
    spi_write_blocking(SPI_PORT, &bank_value, 1);
    gpio_put(IMU_CS_PIN, 1);
}

// Write to a register
void ICM20948::writeRegister(UserBank bank, uint8_t reg, uint8_t value)
{
    selectBank(bank);
    sleep_ms(5);
    gpio_put(IMU_CS_PIN, 0);
    sleep_ms(5);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_write_blocking(SPI_PORT, &value, 1);
    gpio_put(IMU_CS_PIN, 1);
}

// Read from a register
uint8_t ICM20948::readRegister(UserBank bank, uint8_t reg)
{
    uint8_t temp_reg = reg | 0x80;
    uint8_t data;
    selectBank(bank);
    sleep_ms(5);
    gpio_put(IMU_CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &temp_reg, 1);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(IMU_CS_PIN, 1);

    // Use std::cout for output
    std::cout << "Read 0x" << std::hex << std::uppercase << static_cast<int>(data) 
              << " from register 0x" << static_cast<int>(reg) << std::dec << std::endl;
    return data;
}

// Reset IMU to default state
void ICM20948::reset()
{
    writeRegister(UserBank::Bank0, PWR_MGMT_1, 0x81); // Reset chip
    sleep_ms(100);
    writeRegister(UserBank::Bank0, PWR_MGMT_1, 0x01); // Exit sleep mode
    sleep_ms(50);
    writeRegister(UserBank::Bank2, ODR_ALIGN_EN, 0x01); // Align ODR
}

// Read motion data
ICM20948::MotionData ICM20948::readMotionData()
{
    MotionData data;
    uint8_t rx_buffer[18];
    uint8_t reg = ACCEL_XOUT_H | 0x80;
    gpio_put(IMU_CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, rx_buffer, 18);
    gpio_put(IMU_CS_PIN, 1);

    // Parse accelerometer, gyroscope, and magnetometer data
    data.x_accel = (static_cast<int16_t>(rx_buffer[0]) << 8) | rx_buffer[1];
    data.y_accel = (static_cast<int16_t>(rx_buffer[2]) << 8) | rx_buffer[3];
    data.z_accel = (static_cast<int16_t>(rx_buffer[4]) << 8) | rx_buffer[5];
    data.x_gyro = (static_cast<int16_t>(rx_buffer[6]) << 8) | rx_buffer[7];
    data.y_gyro = (static_cast<int16_t>(rx_buffer[8]) << 8) | rx_buffer[9];
    data.z_gyro = (static_cast<int16_t>(rx_buffer[10]) << 8) | rx_buffer[11];
    data.x_mag = (static_cast<int16_t>(rx_buffer[15]) << 8) | rx_buffer[14];
    data.y_mag = (static_cast<int16_t>(rx_buffer[17]) << 8) | rx_buffer[16];
    data.z_mag = (static_cast<int16_t>(rx_buffer[19]) << 8) | rx_buffer[18];

    return data;
}
