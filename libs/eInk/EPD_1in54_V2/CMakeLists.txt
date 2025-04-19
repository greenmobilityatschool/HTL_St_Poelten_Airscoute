# Define the EPD_1in54_V2 library
add_library(epd_1in54_v2 STATIC EPD_1in54_V2.c)

# Include the current directory for this library
target_include_directories(epd_1in54_v2 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Link with pico_stdlib to include Pico SDK headers like hardware/gpio.h
target_link_libraries(epd_1in54_v2 pico_stdlib hardware_spi hardware_i2c hardware_dma hardware_pio hardware_interp hardware_timer hardware_watchdog) 
