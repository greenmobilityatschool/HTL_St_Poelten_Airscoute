#include "pas_co2.h" 
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Constructor to initialize address
Pas_co2::Pas_co2(uint8_t address, i2c_inst_t* i2c_instance) 
    : i2c_address(address), i2c(i2c_instance), result(0) {}

int Pas_co2::init() {
    uint8_t buffer[2];

    // Set sensor to idle mode
    buffer[0] = MEAS_CFG;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);

    // Set measurement rate high and low bytes (10s interval)
    buffer[0] = MEAS_RATE_H;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);
    
    buffer[0] = MEAS_RATE_L;
    buffer[1] = 0x01;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);

    // Set continuous measurement mode
    buffer[0] = MEAS_CFG;
    buffer[1] = 0x02;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);

    return 0;
}

void Pas_co2::read() {
    // Add timeout protection for I2C operations
    absolute_time_t timeout_time = make_timeout_time_ms(1000); // 1 second timeout
    bool timeout_occurred = false;
    data_rdy = 0;  // Reset data ready flag
    
    // Check if new data is available with timeout
    printf("CO2_DEBUG: Starting sensor read operation\n");
    
    int status = i2c_write_timeout_us(i2c, i2c_address, &MEAS_STS, 1, true, 100000); // 100ms timeout
    if (status < 0) {
        printf("CO2_ERROR: I2C write timeout during status check (code: %d)\n", status);
        timeout_occurred = true;
    }
    
    if (!timeout_occurred) {
        status = i2c_read_timeout_us(i2c, i2c_address, &data_rdy, 1, false, 100000); // 100ms timeout
        if (status < 0) {
            printf("CO2_ERROR: I2C read timeout during status check (code: %d)\n", status);
            timeout_occurred = true;
        }
    }

    // Only proceed if no timeout occurred and data is ready
    if (!timeout_occurred && (data_rdy & COMP_BIT)) {
        // Read CO2 PPM high byte with timeout
        status = i2c_write_timeout_us(i2c, i2c_address, &CO2PPM_H, 1, true, 100000);
        if (status < 0) {
            printf("CO2_ERROR: I2C write timeout during high byte read (code: %d)\n", status);
            timeout_occurred = true;
        }
        
        if (!timeout_occurred) {
            status = i2c_read_timeout_us(i2c, i2c_address, &msb, 1, false, 100000);
            if (status < 0) {
                printf("CO2_ERROR: I2C read timeout during high byte read (code: %d)\n", status);
                timeout_occurred = true;
            }
        }

        // Read CO2 PPM low byte with timeout
        if (!timeout_occurred) {
            status = i2c_write_timeout_us(i2c, i2c_address, &CO2PPM_L, 1, true, 100000);
            if (status < 0) {
                printf("CO2_ERROR: I2C write timeout during low byte read (code: %d)\n", status);
                timeout_occurred = true;
            }
            
            if (!timeout_occurred) {
                status = i2c_read_timeout_us(i2c, i2c_address, &lsb, 1, false, 100000);
                if (status < 0) {
                    printf("CO2_ERROR: I2C read timeout during low byte read (code: %d)\n", status);
                    timeout_occurred = true;
                }
            }
        }

        // If we successfully read both bytes, update the result
        if (!timeout_occurred) {
            // Combine high and low bytes to calculate the result
            uint32_t new_result = (msb << 8) | lsb;
            
            // Sanity check - CO2 values should be in a reasonable range (typically 400-5000 ppm)
            if (new_result >= 400 && new_result <= 10000) {
                result = new_result;
                printf("CO2_DEBUG: Valid reading: %u ppm\n", result);
            } else {
                printf("CO2_WARNING: Ignoring suspicious reading: %u ppm (out of expected range)\n", new_result);
            }
        } else {
            printf("CO2_WARNING: Keeping previous reading due to I2C timeout: %u ppm\n", result);
        }
    } else if (timeout_occurred) {
        printf("CO2_ERROR: I2C timeout occurred, keeping previous reading: %u ppm\n", result);
    } else {
        printf("CO2_DEBUG: No new data available (status: 0x%02x), keeping previous reading: %u ppm\n", data_rdy, result);
    }
    
    // If we exceed our overall timeout, ensure we report it
    if (absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0) {
        printf("CO2_ERROR: Overall timeout exceeded (1000ms) - possible I2C bus hang\n");
    }
}
