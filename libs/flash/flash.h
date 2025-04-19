#ifndef FLASH_H
#define FLASH_H

#include <vector>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Add this definition at the top of the file, after the struct definitions
#define SENSOR_DATA_SIZE  sizeof(SensorData)

// SensorData struct matches the one in pico_eu.cpp
struct SensorData {
    float temp = 0.0;
    float hum = 0.0;
    float pres = 0.0;
    float gasRes = 0.0;
    uint16_t pm2_5 = 0;
    uint16_t pm5 = 0;
    uint16_t pm10 = 0;
    uint32_t co2 = 0;
    uint32_t latitude = 0;
    uint32_t longitude = 0;
    uint32_t timestamp = 0;
    bool is_fake_gps = false;  // Flag to indicate if this reading used fake GPS data
};

// Add this struct to ensure aligned, packed serialization
#pragma pack(push, 1)
struct SerializedSensorData {
    // Header with magic number for validation
    uint32_t magic;      // Magic number to validate record integrity (0xABCD1234)
    
    // Environmental data
    float temp;
    float hum;
    float pres;
    float gasRes;
    
    // Particulate matter data
    uint16_t pm2_5;
    uint16_t pm5;
    uint16_t pm10;
    uint16_t padding;  // Ensure proper alignment
    
    // Gas and location data
    uint32_t co2;
    uint32_t latitude;
    uint32_t longitude;
    uint32_t timestamp;
    
    // Flags
    uint8_t flags;       // Bit 0: is_fake_gps, Bits 1-7: reserved for future use
    uint8_t reserved[3]; // Reserved for future expansion, keeps alignment
    
    // Validation checksum
    uint32_t checksum;  // Simple checksum (sum of all values)
};
#pragma pack(pop)

class Flash {
public:
    Flash(uint32_t flash_offset = 0);
    ~Flash();

    // Initialize flash storage
    bool init();
    
    // Save sensor data
    bool saveSensorData(const SensorData& data);
    
    // Save a vector of sensor data points to flash
    bool saveSensorDataBatch(const std::vector<SensorData>& data);
    
    // Load all sensor data
    std::vector<SensorData> loadSensorData();
    
    // Load a single sensor data record
    SensorData loadSensorData(size_t index);
    
    // Load all stored sensor data at once
    std::vector<SensorData> loadAllSensorData();
    
    // Get count of stored records
    uint32_t getStoredCount();
    
    // Get stored data count
    size_t getStoredDataCount() const { return _stored_data_count; }
    
    // Get maximum data count
    uint32_t getMaxDataCount() const { return _max_data_count; }
    
    // Check if storage is full
    bool isStorageFull();
    
    // Erase all user data
    bool eraseStorage();
    
    // Dump raw flash contents for debugging
    void dumpRawFlashContents(size_t max_records);
    
    // Reset storage (erase and reinitialize)
    bool resetStorage();
    
    // Enable or disable flash operations (for testing/debugging)
    void setFlashEnabled(bool enabled) { _flash_enabled = enabled; }
    
    // Check if flash is enabled
    bool isFlashEnabled() const { return _flash_enabled; }
    
    // Set debug verbosity level (0=minimal, 1=normal, 2=verbose)
    void setDebugLevel(int level) { _debug_level = level; }
    
private:
    uint32_t _flash_offset;                // Where to start storing data in flash
    uint32_t _data_count_address;          // Where to store the count of records
    uint32_t _data_start_address;          // Where the actual data starts
    uint32_t _max_data_count;              // Maximum number of records that can be stored
    uint32_t _stored_data_count;           // Current count of stored records
    bool _flash_enabled = true;            // Whether flash operations are enabled
    int _debug_level = 1;                  // Debug verbosity level
    
    // Converts between flash address and XIP mapped address
    inline void* flashAddressToXIP(uint32_t flash_addr) {
        return (void*)(XIP_BASE + flash_addr);
    }
    
    // Serialize sensor data to a byte array
    void serializeSensorData(const SensorData& data, uint8_t* buffer);
    
    // Deserialize sensor data from a byte array
    SensorData deserializeSensorData(const uint8_t* buffer);
    
    // Create an error sensor data record
    SensorData getSensorDataError();
    
    // Read a single record from a specific address
    bool readSensorDataRecord(uint32_t addr, SensorData &data);
    
    // Helper for safe flash operations
    bool safeFlashErase(uint32_t address, size_t size);
    bool safeFlashProgram(uint32_t address, const uint8_t* data, size_t size);
};

#endif // FLASH_H
