#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class Pas_co2 {
public:
    // Constructor to initialize the I2C address and instance
    Pas_co2(uint8_t address, i2c_inst_t* i2c_instance);

    // Public method to initialize the sensor
    int init();

    // Public method to read CO2 concentration from the sensor
    void read();

    // Getter for the CO2 result value
    uint16_t getResult() const { return result; }

private:
    // I2C parameters
    uint8_t i2c_address;
    i2c_inst_t* i2c;

    // Sensor registers and control bits
    const uint8_t MEAS_RATE_H = 0x02;
    const uint8_t MEAS_RATE_L = 0x03;
    const uint8_t MEAS_CFG = 0x04;
    const uint8_t CO2PPM_H = 0x05;
    const uint8_t CO2PPM_L = 0x06;
    const uint8_t MEAS_STS = 0x07;
    const uint8_t COMP_BIT = 0x10;  // Bit 4 indicates unread data availability

    // Sensor data variables
    uint8_t lsb;
    uint8_t msb;
    uint8_t data_rdy;
    uint16_t result;
};
