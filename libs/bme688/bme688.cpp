// bme688.cpp

#include "bme688.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>

BME688::BME688(i2c_inst_t *i2c, uint8_t address, uint8_t sda, uint8_t scl) 
    : i2c_(i2c), address_(address) {
    dev_.intf = BME68X_I2C_INTF;

    dev_.read = [](uint8_t reg, uint8_t *data, uint32_t len, void *intf) -> int8_t {
        BME688 *self = static_cast<BME688 *>(intf);
        int result = i2c_write_blocking(self->i2c_, self->address_, &reg, 1, true);
        if (result < 0) return -1;

        result = i2c_read_blocking(self->i2c_, self->address_, data, len, false);
        return result < 0 ? -1 : 0;
    };

    dev_.write = [](uint8_t reg, const uint8_t *data, uint32_t len, void *intf) -> int8_t {
        BME688 *self = static_cast<BME688 *>(intf);
        uint8_t buf[len + 1];
        buf[0] = reg;
        memcpy(&buf[1], data, len);

        int result = i2c_write_blocking(self->i2c_, self->address_, buf, len + 1, false);
        return result < 0 ? -1 : 0;
    };

    dev_.delay_us = [](uint32_t period, void *intf) {
        sleep_us(period);
    };
    dev_.intf_ptr = this;
}


bool BME688::begin() {
    if (bme68x_init(&dev_) != BME68X_OK) {
        return false;
    }

    // Set configuration in force mode
    conf_.os_hum = BME68X_OS_2X;
    conf_.os_pres = BME68X_OS_4X;
    conf_.os_temp = BME68X_OS_8X;
    conf_.filter = BME68X_FILTER_OFF;
    conf_.odr = BME68X_ODR_NONE;

    if (bme68x_set_conf(&conf_, &dev_) != BME68X_OK) {
        return false;
    }

    // Configure heater for gas measurement in force mode
    heatr_conf_.enable = BME68X_ENABLE;
    heatr_conf_.heatr_temp = 320;     // Target temperature in °C
    heatr_conf_.heatr_dur = 150;      // Heating duration in ms

    return bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf_, &dev_) == BME68X_OK;
}

bool BME688::readData(float &temperature, float &humidity, float &pressure, float &gas_resistance) {
    struct bme68x_data data;
    uint32_t del_period;

    // Apply heater configuration only on the first measurement
    if (!heaterConfigured_) {
        heatr_conf_.enable = BME68X_ENABLE;
        heatr_conf_.heatr_temp = 320;     // Target temperature in °C
        heatr_conf_.heatr_dur = 150;      // Heating duration in ms

        if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf_, &dev_) != BME68X_OK) {
            return false;
        }
        heaterConfigured_ = true;  // Mark the heater as configured
    }

    // Set the sensor to forced mode
    if (bme68x_set_op_mode(BME68X_FORCED_MODE, &dev_) != BME68X_OK) {
        return false;
    }

    // Calculate the minimum delay required and wait
    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf_, &dev_) + heatr_conf_.heatr_dur * 1000;
    dev_.delay_us(del_period, dev_.intf_ptr);

    // Read the data
    uint8_t n_fields;
    if (bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev_) != BME68X_OK) {
        return false;
    }

    // Check if valid data is available
    if (n_fields > 0) {
        temperature = data.temperature;
        humidity = data.humidity;
        pressure = data.pressure / 100.0f; // Convert to hPa
        gas_resistance = data.gas_resistance;
        return true;
    }
    return false;
}