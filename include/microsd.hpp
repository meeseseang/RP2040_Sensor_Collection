#ifndef MICROSD_HPP
#define MICROSD_HPP

#include <ff.h>  // FatFs library
#include <cstdio>
#include <cstdint>
#include <string>

/**
 * @brief MicroSD class to interact with a microSD card on RP2040 using SPI.
 * 
 * Provides file read/write functionality using the FatFs library.
 */
class MicroSD {
public:
    MicroSD();
    ~MicroSD();

    /**
     * @brief Initialize the microSD card and mount the filesystem.
     * @return true if initialization succeeds, false otherwise.
     */
    bool init();

    /**
     * @brief Write data to a specified file.
     * @param filePath Path of the file on the microSD card.
     * @param data String data to write into the file.
     * @return true on success, false on failure.
     */
    bool writeFile(const std::string& filePath, const std::string& data);

    /**
     * @brief Read data from a specified file.
     * @param filePath Path of the file on the microSD card.
     * @param outData Reference to a string where file data will be stored.
     * @return true on success, false on failure.
     */
    bool readFile(const std::string& filePath, std::string& outData);

private:
    FATFS fs;      ///< FatFs filesystem object.
    FIL file;      ///< FatFs file object.
    bool isMounted; ///< Tracks if the filesystem is mounted.

    /**
     * @brief Mount the filesystem if not already mounted.
     * @return true if mounted successfully or already mounted.
     */
    bool mount();

    /**
     * @brief Unmount the filesystem.
     */
    void unmount();
};

#endif // MICROSD_HPP
