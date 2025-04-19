#include "adc.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <cstdio>

// Constructor to initialize the GPIO pin and number of samples for averaging
myADC::myADC(int pin, int samples) : gpioPin(pin), numSamples(samples) {}

// Initialize the ADC for the specified GPIO pin
void myADC::init() const {
    if (gpioPin != 26) {  // Ensure only GPIO 26 (ADC0) is allowed for this instance
        printf("ERROR: Invalid GPIO pin %d. Only GPIO 26 (ADC0) is valid for this instance.\n", gpioPin);
        return;
    }

    adc_init();  // Initialize the ADC hardware
    adc_gpio_init(gpioPin);  // Initialize the specified GPIO pin for ADC use
    adc_select_input(0);  // Select ADC input 0 (corresponding to GPIO 26)
    printf("INFO: ADC initialized on GPIO pin %d (ADC0).\n", gpioPin);
}

// Read a single voltage reading from the ADC pin
float myADC::readVoltage() const {
    uint16_t result = adc_read();  // Read the raw ADC value
    float voltage = result * conversionFactor * 2;  // Adjust for 1:1 voltage divider
    return voltage;
}

// Read and calculate the average voltage from the ADC pin
float myADC::readAverageVoltage() const {
    uint32_t sum = 0;
    for (int i = 0; i < numSamples; ++i) {
        sum += adc_read();  // Sum up the ADC readings
        sleep_ms(1);  // Small delay between readings for stability
    }
    float averageReading = static_cast<float>(sum) / numSamples;
    
    // Debug output to show raw ADC value
    //printf("DEBUG: Raw ADC average reading: %.2f\n", averageReading);

    float voltage = (averageReading * conversionFactor * 2) + offset;  // Adjust for 1:1 voltage divider
    //printf("DEBUG: Converted voltage (before scaling back): %.3fV\n", voltage);
    return voltage;
}

// Calculate the battery level as a percentage based on the voltage
float myADC::calculateBatteryLevel() const {
    float voltage = readAverageVoltage();

    if (voltage < minVoltage) {
        //printf("INFO: Voltage %.3fV is below the minimum threshold. Battery level is 0%%.\n", voltage);
        return 0.0f;
    }

    if (voltage > maxVoltage) {
        //printf("INFO: Voltage %.3fV is above the maximum threshold. Battery level is 100%%.\n", voltage);
        return 100.0f;
    }

    // Calculate the battery level as a percentage
    float batteryLevel = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0f;
    //printf("INFO: Voltage: %.3fV, Battery level: %.2f%%\n", voltage, batteryLevel);
    return batteryLevel;
}
