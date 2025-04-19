# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable C++ exceptions
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_eu C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectories for eInk libraries
add_subdirectory(libs/eInk/EPD_1in54_V2)
add_subdirectory(libs/eInk/Fonts)
add_subdirectory(libs/eInk/GUI)

# Add BME68x library
add_library(BME68x_LIB STATIC libs/bme688/api/bme68x_SensorAPI/bme68x.c)

# Add executable. Default name is the project name, version 0.1
add_executable(pico_eu 
    pico_eu.cpp
    libs/bme688/bme688.cpp
    libs/hm3301/hm3301.cpp
    libs/pas_co2/pas_co2.cpp
    libs/adc/adc.cpp
    libs/wifi/wifi.cpp
    libs/flash/flash.cpp
    libs/eInk/EPD_1in54_V2/EPD_1in54_V2.c    
    libs/eInk/GUI/GUI_Paint.c
    libs/eInk/Fonts/font8.c
    libs/eInk/Fonts/font12.c
    libs/eInk/Fonts/font16.c
    libs/eInk/Fonts/font20.c
    libs/eInk/Fonts/font24.c
    libs/gps/myGPS.cpp
    libs/https/tls.c  # Re-add the TLS implementation
)

pico_set_program_name(pico_eu "pico_eu")
pico_set_program_version(pico_eu "0.1")

# Configure mbedtls and lwip paths
set(LWIP_DIR ${PICO_SDK_PATH}/lib/lwip)
set(MBEDTLS_DIR ${PICO_SDK_PATH}/lib/mbedtls)

# Generate PIO header
pico_generate_pio_header(pico_eu ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_eu 0)
pico_enable_stdio_usb(pico_eu 1)

# Add the standard include files to the build
target_include_directories(pico_eu PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/libs/bme688
    ${CMAKE_CURRENT_LIST_DIR}/libs/bme688/api
    ${CMAKE_CURRENT_LIST_DIR}/libs/hm3301
    ${CMAKE_CURRENT_LIST_DIR}/libs/pas_co2
    ${CMAKE_CURRENT_LIST_DIR}/libs/adc
    ${CMAKE_CURRENT_LIST_DIR}/libs/wifi
    ${CMAKE_CURRENT_LIST_DIR}/libs/flash
    ${CMAKE_CURRENT_LIST_DIR}/libs/eInk
    ${CMAKE_CURRENT_LIST_DIR}/libs/https  # Add HTTPS include directory
    ${LWIP_DIR}/src/include  # Add LWIP include directory
    ${MBEDTLS_DIR}/include  # Add mbedTLS include directory
    ${CMAKE_CURRENT_LIST_DIR}/.. # for common lwipopts or other standard includes
)

# Add the standard library to the build
target_link_libraries(pico_eu
    pico_stdlib
    hardware_adc
    hardware_spi
    hardware_i2c
    hardware_dma
    hardware_pio
    hardware_interp
    hardware_timer
    hardware_watchdog
    hardware_flash
    hardware_sync
    hardware_rtc
    hardware_clocks
    pico_stdio_usb
    pico_cyw43_arch_lwip_threadsafe_background
    BME68x_LIB
    epd_1in54_v2 
    epd_gui_paint 
    epd_fonts 
    pico_flash
    pico_lwip
    pico_lwip_mbedtls  # mbedTLS support for LWIP
    pico_mbedtls       # Add explicit link to mbedTLS
)

pico_add_extra_outputs(pico_eu)

