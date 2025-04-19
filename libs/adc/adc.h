//
// Created by [Your Name] on [Date].
//

#ifndef MY_PROJECT_MYADC_H
#define MY_PROJECT_MYADC_H

#include <cstdio>

class myADC {
private:
    static constexpr float minVoltage = 3.5f;   // Battery level 0%
    static constexpr float maxVoltage = 4.2f;   // Battery level 100%
    static constexpr float conversionFactor = 3.3f / 4095;  // For 12-bit ADC (0-4095)
    float offset = 0.2f;  // Adjust as needed based on measurements

    
    int gpioPin;
    int numSamples;

public:
    explicit myADC(int pin, int samples = 10);  // Constructor with optional sample count
    void init() const;
    float readVoltage() const;
    float readAverageVoltage() const;  // New method to read averaged voltage
    float calculateBatteryLevel() const;
};

#endif // MY_PROJECT_MYADC_H
