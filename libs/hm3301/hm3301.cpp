#include "hm3301.h"
#include "hardware/i2c.h"
#include <cstdio>
#include <cstring> // For memset

// Constructor: Set up I2C parameters for the HM3301 sensor
HM3301::HM3301(i2c_inst_t *i2c_port, uint8_t addr, uint sda_pin, uint scl_pin)
    : i2c_port(i2c_port), addr(addr), sda_pin(sda_pin), scl_pin(scl_pin) {
}

// Initializes the HM3301 sensor
bool HM3301::begin() {
    // Typically no specific initialization command is required for HM3301
    return true; // Return true for successful setup
}

// Reads PM1.0, PM2.5, and PM10 data from the HM3301 sensor
bool HM3301::read(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10) {
    uint8_t data[29];  // HM3301 outputs 29 bytes

    // Read raw data from the sensor
    if (!readRawData(data, sizeof(data))) {
        return false;  // Data read failed
    }

    // Parse PM values from the data buffer
    pm1_0 = (data[6] << 8) | data[7];
    pm2_5 = (data[8] << 8) | data[9];
    pm10 = (data[10] << 8) | data[11];

    return true;
}

// Reads raw data from the HM3301 sensor
bool HM3301::readRawData(uint8_t *data, size_t length) {
    memset(data, 0, length);

    // Perform an I2C read
    int result = i2c_read_blocking(i2c_port, addr, data, length, false);
    return result == length;  // Return true if the full buffer was read successfully
}
