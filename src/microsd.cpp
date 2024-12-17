#include "microsd.hpp"
#include <pico/stdlib.h> // For RP2040 SDK
#include <hardware/spi.h>
#include <cstring>

MicroSD::MicroSD() : isMounted(false) {}

MicroSD::~MicroSD() {
    unmount();
}

bool MicroSD::init() {
    FRESULT res;

    // Initialize the FatFs library and SPI hardware
    if (!isMounted) {
        res = f_mount(&fs, "", 1); // Mount the filesystem
        if (res != FR_OK) {
            printf("MicroSD init failed. Error code: %d\n", res);
            return false;
        }
        isMounted = true;
    }

    printf("MicroSD initialized successfully.\n");
    return true;
}

bool MicroSD::mount() {
    if (!isMounted) {
        FRESULT res = f_mount(&fs, "", 1);
        if (res != FR_OK) {
            printf("Failed to mount microSD card. Error code: %d\n", res);
            return false;
        }
        isMounted = true;
    }
    return true;
}

void MicroSD::unmount() {
    if (isMounted) {
        f_unmount("");
        isMounted = false;
    }
}

bool MicroSD::writeFile(const std::string& filePath, const std::string& data) {
    if (!mount()) return false;

    FRESULT res = f_open(&file, filePath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("Failed to open file for writing. Error code: %d\n", res);
        return false;
    }

    UINT bytesWritten;
    res = f_write(&file, data.c_str(), data.size(), &bytesWritten);
    if (res != FR_OK || bytesWritten != data.size()) {
        printf("Failed to write data. Error code: %d\n", res);
        f_close(&file);
        return false;
    }

    f_close(&file);
    printf("File written successfully: %s\n", filePath.c_str());
    return true;
}

bool MicroSD::readFile(const std::string& filePath, std::string& outData) {
    if (!mount()) return false;

    FRESULT res = f_open(&file, filePath.c_str(), FA_READ);
    if (res != FR_OK) {
        printf("Failed to open file for reading. Error code: %d\n", res);
        return false;
    }

    constexpr size_t bufferSize = 512;
    char buffer[bufferSize];
    UINT bytesRead;
    outData.clear();

    // Read file contents into buffer
    do {
        res = f_read(&file, buffer, bufferSize, &bytesRead);
        if (res != FR_OK) {
            printf("Failed to read file. Error code: %d\n", res);
            f_close(&file);
            return false;
        }
        outData.append(buffer, bytesRead);
    } while (bytesRead == bufferSize);

    f_close(&file);
    printf("File read successfully: %s\n", filePath.c_str());
    return true;
}
