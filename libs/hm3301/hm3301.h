#ifndef HM3301_H
#define HM3301_H

#include <stdint.h>
#include "hardware/i2c.h"

class HM3301 {
public:
    HM3301(i2c_inst_t *i2c_port, uint8_t addr, uint sda_pin, uint scl_pin);
    bool begin();
    bool read(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10);

private:
    i2c_inst_t *i2c_port;
    uint8_t addr;
    uint sda_pin;
    uint scl_pin;

    bool readRawData(uint8_t *data, size_t length);
};

#endif // HM3301_H
