# Minimum CMake version
cmake_minimum_required(VERSION 3.16)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)


# Project name and version
project(RP2040_Flight_Computer_CPP VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Creates a pico-sdk subdirectory in the project for the librarcmies
pico_sdk_init()

# Set the output directory for build files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_NAME}/bin)

# Include directories
include_directories(${PROJECT_NAME}
    ./include/
    )


# Add the executable target
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/icm_20948.cpp
    src/gpio_init.cpp
    src/buzzer.cpp
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc)
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_spi
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)