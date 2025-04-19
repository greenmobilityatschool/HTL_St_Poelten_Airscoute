// bme688.h

#ifndef BME688_H
#define BME688_H

#include "hardware/i2c.h"
#include "libs/bme688/api/BME68x_SensorAPI/bme68x.h"  // Include Bosch's sensor API

class BME688 {
public:
    BME688(i2c_inst_t *i2c, uint8_t address, uint8_t sda, uint8_t scl);
    bool begin();
    bool readData(float &temperature, float &humidity, float &pressure, float &gas_resistance);

private:
    i2c_inst_t *i2c_;
    uint8_t address_;
    struct bme68x_dev dev_;
    struct bme68x_conf conf_;
    struct bme68x_heatr_conf heatr_conf_;

    bool heaterConfigured_ = true;  // Flag to track heater configuration
};

#endif // BME688_H
