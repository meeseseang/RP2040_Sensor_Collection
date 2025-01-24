#include "icm_20948.hpp"
#include "buzzer.hpp"

// Constructor
ICM20948::ICM20948(int CS, int MISO, int MOSI, int SCLK, spi_inst_t* SPI)
{
    IMU_CS_PIN = CS;
    SPI_PORT = SPI;

    // Init gpio pins
    gpio_init(IMU_CS_PIN);
    gpio_set_dir(IMU_CS_PIN, GPIO_OUT);
    gpio_put(IMU_CS_PIN, 1); // Set CS high initially

    // SPI0 interface initialization
    spi_init(SPI_PORT, 1000000);
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    spi_set_format( SPI_PORT,   // SPI instance
                    8,      //Number of bits per transfer
                    SPI_CPOL_1,      // Polarity (CPOL)
                    SPI_CPHA_1,      // Phase (CPHA)
                    SPI_MSB_FIRST);
}

// Read WHO_AM_I register
uint8_t ICM20948::whoAmI()
{
    uint8_t reg = IMU_DEV_ID | 0x80;
    uint8_t data;
    gpio_put(IMU_CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(IMU_CS_PIN, 1);
    return data;
}

// Select user bank
void ICM20948::selectBank(UserBank bank)
{
    gpio_put(IMU_CS_PIN, 0);
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
    sleep_ms(10);
    gpio_put(IMU_CS_PIN, 0);
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
    sleep_ms(10);
    gpio_put(IMU_CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &temp_reg, 1);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(IMU_CS_PIN, 1);
    return data;
}

// Initialize magnetometer
void ICM20948::initMagnetometer()
{
    uint8_t temp_data;

    // I2C master reset (pg.36)
    temp_data = readRegister(UserBank::Bank0, USER_CTRL);
    temp_data |= 0x02;
    writeRegister(UserBank::Bank0, USER_CTRL, temp_data);
    sleep_ms(100);

    // I2C Master enable, (pg.36)
    temp_data = readRegister(UserBank::Bank0, USER_CTRL);
    temp_data |= 0x20;
    writeRegister(UserBank::Bank0, USER_CTRL, temp_data);
    sleep_ms(10);

    // I2C Master clock: 7 (400kHz), (pg.68)
    writeRegister(UserBank::Bank3, I2C_MST_CTRL, 0x07);
    sleep_ms(10);

    // LP Config: Output Data Rate (pg.37)
    writeRegister(UserBank::Bank0, LP_CONFIG, 0x40);
    sleep_ms(10);

    // I2C MST ODR config: 1.1kHz/(2^ODR) = Data Rate(Hz), (pg.68)
    writeRegister(UserBank::Bank3, I2C_MST_ODR_CONFIG, ODR);
    sleep_ms(10);

    // Magnetometer reset, (pg.80)
    writeMagRegister(MAG_CTRL3, 0x01);
    sleep_ms(100);

    // Set the magnetometer to continuous mode 4: 100Hz, (pg.79)
    writeMagRegister(MAG_CTRL2, 0x08);
}

// Write to mag
void ICM20948::writeMagRegister(uint8_t reg, uint8_t data)
{
    writeRegister(UserBank::Bank3, I2C_SLV0_ADDR, AK09916_ADDRESS);
    writeRegister(UserBank::Bank3, I2C_SLV0_REG,  reg);
    writeRegister(UserBank::Bank3, I2C_SLV0_D0,   data);
    writeRegister(UserBank::Bank3, I2C_SLV0_CTRL, 0x80|0x01);
}

// Read from mag
void ICM20948::readMagRegister(uint8_t onsetReg, uint8_t len)
{
    writeRegister(UserBank::Bank3, I2C_SLV0_ADDR, 0x80|AK09916_ADDRESS);
    writeRegister(UserBank::Bank3, I2C_SLV0_REG,  onsetReg);
    writeRegister(UserBank::Bank3, I2C_SLV0_CTRL, 0x80|len);
}

// Remove gyro bias
void ICM20948::removeGyroBias()
{
    int16_t x_gyro_bias, y_gyro_bias, z_gyro_bias;
    MotionData data;
    int32_t x_bias = 0, y_bias = 0, z_bias = 0;

    // Gather 100 samples to determine gyro bias
    for (int i = 0; i < BIAS_SAMPLES; i++)
    {
        data = readMotionData();
        x_bias += (int32_t)data.x_gyro;
        y_bias += (int32_t)data.y_gyro;
        z_bias += (int32_t)data.z_gyro;
        sleep_ms(2);
    }

    // Determine bias
    x_gyro_bias = -(int16_t)(x_bias / BIAS_RATE);
    y_gyro_bias = -(int16_t)(y_bias / BIAS_RATE);
    z_gyro_bias = -(int16_t)(z_bias / BIAS_RATE);
    sleep_ms(10);

    // Write bias to registers to filter it out
    writeRegister(UserBank::Bank2, XG_OFFS_USRH, (uint8_t)(x_gyro_bias >> 8));
    writeRegister(UserBank::Bank2, XG_OFFS_USRL, (uint8_t)(x_gyro_bias));

    writeRegister(UserBank::Bank2, YG_OFFS_USRH, (uint8_t)(y_gyro_bias >> 8));
    writeRegister(UserBank::Bank2, YG_OFFS_USRL, (uint8_t)(x_gyro_bias));

    writeRegister(UserBank::Bank2, ZG_OFFS_USRH, (uint8_t)(z_gyro_bias >> 8));
    writeRegister(UserBank::Bank2, ZG_OFFS_USRL, (uint8_t)(z_gyro_bias));
}

// Calibrate magnetometer
void ICM20948::calibrateMagnetometer()
{}

// Reset IMU to default state
void ICM20948::reset()
{
    // Beep to signal reset
    Buzzer::long_beep;

    uint8_t temp_data;

    // Resets chip (pg. 37)
    writeRegister(UserBank::Bank0, PWR_MGMT_1, 0x81);
    sleep_ms(10);

    // Exit from sleep mode, selecting clock (pg.37)
    writeRegister(UserBank::Bank0, PWR_MGMT_1, 0x01);

    // Start sensor alignment (pg.63)
    writeRegister(UserBank::Bank2, ODR_ALIGN_EN, 0x01);
    sleep_ms(50);

    // Accelerometer sample rate divider = 0 (pg.63)
    writeRegister(UserBank::Bank2, ACCEL_SMPLRT_DIV_1, ACCEL_SAMPLE_RATE_MSB);
    writeRegister(UserBank::Bank2, ACCEL_SMPLRT_DIV_2, ACCEL_SAMPLE_RATE_LSB);

    // Accelerometer range set and enable digital filter (pg.64)
    writeRegister(UserBank::Bank2, ACCEL_CONFIG, ((ACCEL_RANGE_VALUE << 1)|0x01));

    // Put the serial interface to SPI only mode (pg.36)
    temp_data = readRegister(UserBank::Bank0, USER_CTRL);
    temp_data |= 0x10;
    writeRegister(UserBank::Bank0, USER_CTRL, temp_data);

    // Initialize the magnetometer
    initMagnetometer();
    readMagRegister(MAG_DATA_ONSET, 8);
    sleep_ms(50);

    selectBank(UserBank::Bank0);

    // Alert the beginning of gyro bias removal and leave board still
    Buzzer::long_beep();
    sleep_ms(1500);
    Buzzer::multi_beep(3, 200);

    // Remove gyro bias
    removeGyroBias();
    sleep_ms(50);

    // Gyro sample rate divider = 0 (pg.59)
    writeRegister(UserBank::Bank2, GYRO_SMPLRT_DIV, GYRO_SAMPLE_RATE);

    // Gyroscope range set and enable digital filter
    writeRegister(UserBank::Bank2, GYRO_CONFIG_1, ((GYRO_SENSITIVITY << 1)|0x01));

    // Alert completion of gyro calibration
    Buzzer::long_beep();
    sleep_ms(2000);

    // Alert the beginning of magnetometer calibration
    Buzzer::long_beep();
    sleep_ms(1500);
    Buzzer::multi_beep(4, 200);

    // Remove mag bias
    calibrateMagnetometer();
    sleep_ms(100);

    // Alert completion of initialization
    Buzzer::long_beep();
    selectBank(UserBank::Bank0);
    sleep_ms(500);
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
