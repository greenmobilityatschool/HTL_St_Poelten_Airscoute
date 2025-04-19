#include "flash.h"
#include <cstring>
#include <algorithm>
#include <cstdio>  // Add this include for printf
#include <cmath>  // For fabs()
#include <string.h>
#include "pico/unique_id.h"

// Constants - Using the SDK's macros directly instead of redefining them
// (removes the redefinition warnings)
#define DATA_COUNT_SIZE   sizeof(uint32_t)

// Default flash target offset (1.8MB from beginning of flash)
#define DEFAULT_FLASH_TARGET_OFFSET (1792 * 1024)

Flash::Flash(uint32_t flash_offset) {
    if (flash_offset == 0) {
        _flash_offset = DEFAULT_FLASH_TARGET_OFFSET;
    } else {
        _flash_offset = flash_offset;
    }
    
    // Ensure flash_offset is sector-aligned
    if (_flash_offset % FLASH_SECTOR_SIZE != 0) {
        uint32_t aligned_offset = (_flash_offset / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
        printf("FLASH: Aligning flash offset to sector boundary: 0x%08x -> 0x%08x\n", 
               (unsigned int)_flash_offset, (unsigned int)aligned_offset);
        _flash_offset = aligned_offset;
    }
    
    // Setup addresses - ensure sector alignment
    _data_count_address = _flash_offset;
    
    // Calculate sector-aligned data start address
    // Use a full sector for the count information
    _data_start_address = _flash_offset + FLASH_SECTOR_SIZE;
    
    // Calculate max data count
    // Increase storage from 16 sectors to 32 sectors for more capacity
    uint32_t storage_size = 32 * FLASH_SECTOR_SIZE - FLASH_SECTOR_SIZE;
    _max_data_count = storage_size / SENSOR_DATA_SIZE;
    
    _stored_data_count = 0;
    
    printf("FLASH: Storage initialized with offset 0x%08x, data start 0x%08x, capacity %lu records\n",
           (unsigned int)_flash_offset, (unsigned int)_data_start_address, _max_data_count);
}

Flash::~Flash() {
    // Nothing to clean up
}

bool Flash::init() {
    printf("FLASH: Initializing flash storage at address 0x%08x\n", (unsigned int)_flash_offset);
    printf("FLASH: Debug level = %d, Flash enabled = %s\n", 
           _debug_level, _flash_enabled ? "true" : "false");
    
    if (!_flash_enabled) {
        printf("FLASH: [DISABLED] Operating in memory-only mode (no flash writes)\n");
        _stored_data_count = 0;
        return true;
    }
    
    // Ensure addresses are sector-aligned for operations
    uint32_t count_sector_offset = _data_count_address % FLASH_SECTOR_SIZE;
    uint32_t count_sector_address = _data_count_address - count_sector_offset;
    
    // Recalculate data start address to be after count sector
    uint32_t aligned_data_start = count_sector_address + FLASH_SECTOR_SIZE;
    
    // If the original _data_start_address isn't aligned, update it
    if (_data_start_address != aligned_data_start) {
        printf("FLASH: Adjusting data start address for sector alignment: 0x%08x -> 0x%08x\n",
               (unsigned int)_data_start_address, (unsigned int)aligned_data_start);
        _data_start_address = aligned_data_start;
    }
    
    // Read stored count from flash
    const uint32_t* stored_count_ptr = (const uint32_t*)flashAddressToXIP(_data_count_address);
    uint32_t stored_count = *stored_count_ptr;
    
    printf("FLASH: Read stored count: %lu (0x%08x)\n", stored_count, stored_count);
    
    // Check if we should force a full storage reset for debugging purposes
    #if defined(FORCE_FLASH_RESET) && FORCE_FLASH_RESET == 1
    printf("FLASH: FORCE_FLASH_RESET defined, performing full reset\n");
    resetStorage();
    stored_count = 0;
    #endif
    
    // Validate count (simple check to see if flash is initialized)
    if (stored_count == 0xFFFFFFFF) { // Erased flash is all 1's
        printf("FLASH: First-time initialization (all 0xFF)\n");
        _stored_data_count = 0;
        
        // Initialize by writing 0 as the count
        uint32_t count = 0;
        
        // Erase the page containing the count using our safe function
        printf("FLASH: Initializing count at 0x%08x\n", (unsigned int)_data_count_address);
        
        // Use our safe erase function that handles failures better
        if (!safeFlashErase(_data_count_address, FLASH_PAGE_SIZE)) {
            printf("FLASH ERROR: Failed to erase count page during initialization\n");
            printf("FLASH: Continuing with in-memory only mode (_stored_data_count = 0)\n");
            _stored_data_count = 0;
            _flash_enabled = false;  // Disable flash operations after failure
            return true;  // Return true to continue with in-memory mode
        }
        
        // Program the count (0) using our safe function
        if (!safeFlashProgram(_data_count_address, (const uint8_t*)&count, DATA_COUNT_SIZE)) {
            printf("FLASH ERROR: Failed to program count during initialization\n");
            printf("FLASH: Continuing with in-memory only mode (_stored_data_count = 0)\n");
            _stored_data_count = 0;
            _flash_enabled = false;  // Disable flash operations after failure
            return true;  // Return true to continue with in-memory mode
        }
        
        printf("FLASH: First-time initialization completed successfully\n");
    } else if (stored_count > _max_data_count) {
        printf("FLASH ERROR: Invalid count (%lu) exceeds maximum (%lu), resetting storage\n", 
               stored_count, _max_data_count);
        
        if (!resetStorage()) {
            printf("FLASH ERROR: Storage reset failed, continuing with in-memory only mode\n");
            _stored_data_count = 0;
            _flash_enabled = false;  // Disable flash operations after failure
            return true;  // Return true to continue with in-memory mode
        }
        
        _stored_data_count = 0;
        return true;
    } else {
        printf("FLASH: Found valid count: %lu\n", stored_count);
        _stored_data_count = stored_count;
        
        // Verify if actual data exists and matches the count
        bool valid_data_found = false;
        if (_stored_data_count > 0) {
            // Check the first record to see if it has valid data
            uint32_t test_address = _data_start_address;
            const uint32_t* magic_ptr = (const uint32_t*)flashAddressToXIP(test_address);
            printf("FLASH: Testing first record at 0x%08x, magic=0x%08x\n", 
                  (unsigned int)test_address, (unsigned int)*magic_ptr);
                  
            if (*magic_ptr == 0xABCD1234) {
                valid_data_found = true;
                printf("FLASH: Found valid data signature\n");
            } else if (*magic_ptr == 0xFFFFFFFF) {
                printf("FLASH WARNING: Count > 0 but data area contains 0xFF values\n");
                printf("FLASH WARNING: Data may be corrupted or missing\n");
                
                printf("FLASH: Detected inconsistency between count and data, resetting storage\n");
                if (!resetStorage()) {
                    printf("FLASH ERROR: Storage reset failed, continuing with in-memory only mode\n");
                    _stored_data_count = 0;
                    _flash_enabled = false;  // Disable flash operations after failure
                    return true;  // Return true to continue with in-memory mode
                }
                return true;
            } else {
                printf("FLASH WARNING: Count > 0 but data has invalid magic number: 0x%08x\n", 
                       (unsigned int)*magic_ptr);
                printf("FLASH: Detected corrupted data, resetting storage\n");
                
                if (!resetStorage()) {
                    printf("FLASH ERROR: Storage reset failed, continuing with in-memory only mode\n");
                    _stored_data_count = 0;
                    _flash_enabled = false;  // Disable flash operations after failure
                    return true;  // Return true to continue with in-memory mode
                }
                return true;
            }
        }
    }
    
    printf("FLASH: Initialization complete. Storage can hold %lu records, %lu currently stored.\n", 
           _max_data_count, _stored_data_count);
    
    // Dump the first few bytes of the first record for diagnosis
    if (_stored_data_count > 0) {
        const uint8_t* data_ptr = (const uint8_t*)flashAddressToXIP(_data_start_address);
        printf("FLASH: First record data preview: ");
        for (int i = 0; i < 16; i++) {
            printf("%02x ", data_ptr[i]);
        }
        printf("\n");
    }
    
    return true;
}

bool Flash::saveSensorData(const SensorData& data) {
    if (!_flash_enabled) {
        if (_debug_level > 0) {
            printf("FLASH: [DISABLED] Skipping sensor data save (operating in memory-only mode)\n");
        }
        return true;
    }
    
    // Check if there's space
    if (_stored_data_count >= _max_data_count) {
        printf("FLASH ERROR: No space left for new records (max: %lu)\n", _max_data_count);
        return false;
    }
    
    // Calculate storage address for this data point
    uint32_t data_address = _data_start_address + (_stored_data_count * SENSOR_DATA_SIZE);
    
    // Check if this write would cross a page boundary
    uint32_t page_offset = data_address % FLASH_PAGE_SIZE;
    uint32_t page_address = data_address - page_offset; // Get the start of the page
    
    // Calculate sector-aligned address for erasing
    uint32_t sector_offset = page_address % FLASH_SECTOR_SIZE;
    uint32_t sector_address = page_address - sector_offset;
    
    // For safety, ensure we're in a valid region of flash
    // Instead of checking against _data_start_address, check against overall flash area
    if (sector_address < _flash_offset) {
        printf("FLASH ERROR: Invalid sector address calculation: 0x%08x is below flash storage area (0x%08x)\n", 
               (unsigned int)sector_address, (unsigned int)_flash_offset);
        return false;
    }
    
    // Also check we're not too far beyond our allocated storage
    uint32_t max_allowed_address = _flash_offset + (16 * FLASH_SECTOR_SIZE);
    if (sector_address >= max_allowed_address) {
        printf("FLASH ERROR: Invalid sector address calculation: 0x%08x exceeds allocated flash area (0x%08x)\n", 
               (unsigned int)sector_address, (unsigned int)max_allowed_address);
        return false;
    }
    
    if (page_offset + SENSOR_DATA_SIZE > FLASH_PAGE_SIZE) {
        if (_debug_level > 0) {
            printf("FLASH: Adjusting to prevent crossing page boundary\n");
        }
        // Skip to the next page to avoid crossing page boundary
        page_address += FLASH_PAGE_SIZE;
        page_offset = 0;
        data_address = page_address; // Start at beginning of next page
        
        // Recalculate sector alignment for the new page address
        sector_offset = page_address % FLASH_SECTOR_SIZE;
        sector_address = page_address - sector_offset;
    }
    
    if (_debug_level > 0) {
        printf("FLASH: Writing record at address 0x%08x (page 0x%08x + offset %u)\n", 
               (unsigned int)data_address, (unsigned int)page_address, (unsigned int)page_offset);
        printf("FLASH: Using sector-aligned address 0x%08x for erase operation\n", 
               (unsigned int)sector_address);
    }
    
    // Allocate a buffer for the entire sector (not just the page)
    uint8_t* sector_buffer = new uint8_t[FLASH_SECTOR_SIZE];
    if (!sector_buffer) {
        printf("FLASH ERROR: Failed to allocate sector buffer\n");
        return false;
    }
    
    // Read current contents of the sector - we need to preserve other data
    const uint8_t* existing_sector = (const uint8_t*)flashAddressToXIP(sector_address);
    memcpy(sector_buffer, existing_sector, FLASH_SECTOR_SIZE);
    
    // Calculate the offset within the sector where our page is located
    uint32_t page_offset_in_sector = page_address - sector_address;
    
    // Calculate where in the buffer to place our data
    uint32_t data_offset_in_buffer = page_offset_in_sector + page_offset;
    
    // Serialize sensor data directly into the sector buffer at the right offset
    serializeSensorData(data, sector_buffer + data_offset_in_buffer);
    
    // Use our safe erase function on the sector-aligned address
    if (!safeFlashErase(sector_address, FLASH_SECTOR_SIZE)) {
        printf("FLASH ERROR: Failed to erase sector during saveSensorData\n");
        delete[] sector_buffer;
        return false;
    }
    
    // Use our safe program function for the entire sector
    if (!safeFlashProgram(sector_address, sector_buffer, FLASH_SECTOR_SIZE)) {
        printf("FLASH ERROR: Failed to program sector during saveSensorData\n");
        delete[] sector_buffer;
        return false;
    }
    
    delete[] sector_buffer;
    
    // Verify the data was written correctly
    const uint8_t* verification_ptr = (const uint8_t*)flashAddressToXIP(data_address);
    if (_debug_level > 0) {
        printf("FLASH VERIFICATION: Raw bytes at 0x%08x: ", (unsigned int)data_address);
        for (size_t i = 0; i < 16 && i < SENSOR_DATA_SIZE; i++) {
            printf("%02x ", verification_ptr[i]);
        }
        printf("\n");
    }
    
    // Double-check the magic number directly
    uint32_t written_magic = *((const uint32_t*)verification_ptr);
    if (written_magic != 0xABCD1234) {
        printf("FLASH ERROR: Magic number verification failed! Expected 0xABCD1234, got 0x%08x\n", 
               written_magic);
        return false;
    }
    
    // Read back for verification
    SensorData verify = deserializeSensorData(verification_ptr);
    if (_debug_level > 0) {
        printf("FLASH VERIFICATION: Read back: Time=%u, Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u\n",
               verify.timestamp, verify.temp, verify.hum, verify.co2, verify.pm2_5);
    }
    
    // Verify a few key fields match what we intended to write
    bool verification_failed = false;
    if (std::fabs(verify.temp - data.temp) > 0.01) {
        printf("FLASH ERROR: Temperature verification failed! %.2f != %.2f\n", 
               verify.temp, data.temp);
        verification_failed = true;
    }
    if (std::fabs(verify.hum - data.hum) > 0.01) {
        printf("FLASH ERROR: Humidity verification failed! %.2f != %.2f\n", 
               verify.hum, data.hum);
        verification_failed = true;
    }
    if (verify.co2 != data.co2) {
        printf("FLASH ERROR: CO2 verification failed! %u != %u\n", 
               verify.co2, data.co2);
        verification_failed = true;
    }
    if (verify.timestamp != data.timestamp) {
        printf("FLASH ERROR: Timestamp verification failed! %u != %u\n", 
               verify.timestamp, data.timestamp);
        verification_failed = true;
    }
    
    if (verification_failed) {
        printf("FLASH ERROR: Data verification failed, record may be corrupted\n");
        return false;
    }
    
    // Update stored count and save to flash
    _stored_data_count++;
    if (_debug_level > 0) {
        printf("FLASH: Successfully saved record %lu\n", _stored_data_count - 1);
    }
    
    // Now update the count in flash
    uint32_t count_buffer = _stored_data_count;
    
    // For count page, ensure it's also sector-aligned
    uint32_t count_sector_offset = _data_count_address % FLASH_SECTOR_SIZE;
    uint32_t count_sector_address = _data_count_address - count_sector_offset;
    
    // Read current sector, update count, then write back
    uint8_t* count_sector_buffer = new uint8_t[FLASH_SECTOR_SIZE];
    if (!count_sector_buffer) {
        printf("FLASH ERROR: Failed to allocate count sector buffer\n");
        // Revert the in-memory count to match what was previously in flash
        _stored_data_count--;
        return false;
    }
    
    // Read existing sector
    const uint8_t* existing_count_sector = (const uint8_t*)flashAddressToXIP(count_sector_address);
    memcpy(count_sector_buffer, existing_count_sector, FLASH_SECTOR_SIZE);
    
    // Update count in buffer
    uint32_t count_offset_in_sector = _data_count_address - count_sector_address;
    memcpy(count_sector_buffer + count_offset_in_sector, &count_buffer, DATA_COUNT_SIZE);
    
    // Use our safe erase function for count sector
    if (!safeFlashErase(count_sector_address, FLASH_SECTOR_SIZE)) {
        printf("FLASH ERROR: Failed to erase count sector while updating count\n");
        delete[] count_sector_buffer;
        // Revert the in-memory count to match what was previously in flash
        _stored_data_count--;
        return false;
    }
    
    // Use our safe program function to update the count
    if (!safeFlashProgram(count_sector_address, count_sector_buffer, FLASH_SECTOR_SIZE)) {
        printf("FLASH ERROR: Failed to program count while updating count\n");
        delete[] count_sector_buffer;
        // Revert the in-memory count to match what was previously in flash
        _stored_data_count--;
        return false;
    }
    
    delete[] count_sector_buffer;
    
    // Verify the count was written correctly
    const uint32_t* verify_count = (const uint32_t*)flashAddressToXIP(_data_count_address);
    if (*verify_count != _stored_data_count) {
        printf("FLASH ERROR: Count verification failed! %lu != %lu\n", 
               *verify_count, _stored_data_count);
        
        // Revert the in-memory count to match what was previously in flash
        _stored_data_count = *verify_count;
        return false;
    }
    
    return true;
}

bool Flash::saveSensorDataBatch(const std::vector<SensorData>& data) {
    // Check if there's enough space
    if (_stored_data_count + data.size() > _max_data_count) {
        return false;
    }
    
    // Calculate how many pages we need
    size_t totalSize = data.size() * SENSOR_DATA_SIZE;
    size_t pages = (totalSize + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    
    // Serialize all data into a buffer
    uint8_t* buffer = new uint8_t[pages * FLASH_PAGE_SIZE];
    memset(buffer, 0xFF, pages * FLASH_PAGE_SIZE); // Initialize with 0xFF (erased flash value)
    
    for (size_t i = 0; i < data.size(); i++) {
        serializeSensorData(data[i], buffer + (i * SENSOR_DATA_SIZE));
    }
    
    // Calculate storage address for this batch
    uint32_t data_address = _data_start_address + (_stored_data_count * SENSOR_DATA_SIZE);
    
    // Write data to flash
    uint32_t interrupt_state = save_and_disable_interrupts();
    flash_range_program(data_address, buffer, pages * FLASH_PAGE_SIZE);
    restore_interrupts(interrupt_state);
    
    delete[] buffer;
    
    // Update count in flash
    _stored_data_count += data.size();
    uint32_t interrupt_state2 = save_and_disable_interrupts();
    flash_range_program(_data_count_address, (const uint8_t*)&_stored_data_count, DATA_COUNT_SIZE);
    restore_interrupts(interrupt_state2);
    
    return true;
}

SensorData Flash::loadSensorData(size_t index) {
    if (index >= _stored_data_count) {
        printf("FLASH ERROR: Attempted to read index %lu but only %lu records stored\n", 
               index, _stored_data_count);
        return getSensorDataError();
    }
    
    uint32_t address = _data_start_address + (index * SENSOR_DATA_SIZE);
    printf("FLASH DEBUG: Loading record %lu from address 0x%08x\n", index, (unsigned int)address);
    
    // Validate this address is within our flash data region
    if (address < _data_start_address || 
        address >= (_data_start_address + (_max_data_count * SENSOR_DATA_SIZE))) {
        printf("FLASH ERROR: Address 0x%08x is outside valid data range\n", (unsigned int)address);
        return getSensorDataError();
    }
    
    // Get a pointer to the flash memory at the calculated address
    const uint8_t* data_ptr = (const uint8_t*)flashAddressToXIP(address);
    
    // Print first 16 bytes for debug
    printf("FLASH DEBUG: Raw bytes for record %lu: ", index);
    for (size_t i = 0; i < 16 && i < SENSOR_DATA_SIZE; i++) {
        printf("%02x ", data_ptr[i]);
    }
    printf("\n");
    
    // Quick check for erased flash (all 0xFF)
    bool is_erased = true;
    for (size_t i = 0; i < 16 && i < SENSOR_DATA_SIZE; i++) {
        if (data_ptr[i] != 0xFF) {
            is_erased = false;
            break;
        }
    }
    
    if (is_erased) {
        printf("FLASH ERROR: Record %lu appears to be erased (all 0xFF)\n", index);
        return getSensorDataError();
    }
    
    // Check magic number directly from the serialized data
    uint32_t magic = *((const uint32_t*)data_ptr);
    if (magic != 0xABCD1234) {
        printf("FLASH ERROR: Invalid magic number in record %lu: 0x%08x\n", 
               index, magic);
        return getSensorDataError();
    }
    
    // Deserialize and return the data
    SensorData result = deserializeSensorData(data_ptr);
    
    // Validate deserialized data for reasonableness
    if (result.temp < -50.0f || result.temp > 100.0f || 
        result.hum < 0.0f || result.hum > 100.0f || 
        result.timestamp < 1600000000 || result.timestamp > 2000000000) {
        printf("FLASH ERROR: Deserialized data contains suspicious values:");
        printf(" Time=%u, Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u\n",
               result.timestamp, result.temp, result.hum, result.co2, result.pm2_5);
    } else {
        printf("FLASH DEBUG: Successfully loaded record %lu: Time=%u, Temp=%.2f, Hum=%.2f\n", 
               index, result.timestamp, result.temp, result.hum);
    }
    
    return result;
}

/**
 * Load all sensor data records from flash storage
 * 
 * @return Vector containing all valid sensor data records
 */
std::vector<SensorData> Flash::loadAllSensorData() {
    std::vector<SensorData> result;
    
    // Return empty vector if no data or flash disabled
    if (!_flash_enabled || getStoredCount() == 0) {
        printf("FLASH: No records found to load or flash disabled\n");
        return result;
    }
    
    // Reserve space for all records
    uint32_t count = getStoredCount();
    result.reserve(count);
    printf("FLASH: Loading %lu records from flash\n", count);
    
    // Read each record using the single-record method
    for (size_t i = 0; i < count; i++) {
        SensorData data = loadSensorData(i);
        
        // Only add valid records to the vector (check timestamp as a validity indicator)
        if (data.timestamp != 0) {
            result.push_back(data);
        } else {
            printf("FLASH WARNING: Skipping invalid record at index %lu\n", i);
        }
    }
    
    printf("FLASH: Successfully loaded %lu valid records (out of %lu total)\n", 
           result.size(), count);
    return result;
}

bool Flash::eraseStorage() {
    printf("FLASH: Erasing flash storage...\n");
    
    // Calculate total sectors to erase (16 sectors = 64KB by default)
    uint32_t sectors_to_erase = 16; // Using a fixed number for safety
    
    // Erase sectors (count page and data pages)
    uint32_t interrupt_state = save_and_disable_interrupts();
    printf("FLASH: Erasing %u sectors starting at 0x%08x\n", 
           (unsigned int)sectors_to_erase, (unsigned int)_flash_offset);
    flash_range_erase(_flash_offset, sectors_to_erase * FLASH_SECTOR_SIZE);
    restore_interrupts(interrupt_state);
    
    // Reset count to 0
    _stored_data_count = 0;
    uint32_t count = 0;
    
    // Program count (0) to flash
    uint32_t interrupt_state2 = save_and_disable_interrupts();
    printf("FLASH: Writing count (0) after erase\n");
    flash_range_program(_data_count_address, (const uint8_t*)&count, DATA_COUNT_SIZE);
    restore_interrupts(interrupt_state2);
    
    // Verify count was written correctly
    const uint32_t* verify_count = (const uint32_t*)flashAddressToXIP(_data_count_address);
    if (*verify_count != 0) {
        printf("FLASH ERROR: Failed to reset count after erase!\n");
        return false;
    }
    
    printf("FLASH: Storage erased successfully. Ready for new records.\n");
    return true;
}

uint32_t Flash::getStoredCount() {
    return _stored_data_count;
}

bool Flash::isStorageFull() {
    return _stored_data_count >= _max_data_count;
}

void Flash::serializeSensorData(const SensorData& data, uint8_t* buffer) {
    // Add detailed debugging to track serialization issues
    printf("SERIALIZING: Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u, Timestamp=%u, FakeGPS=%s\n",
           data.temp, data.hum, data.co2, data.pm2_5, data.timestamp, 
           data.is_fake_gps ? "true" : "false");
    
    // Create a packed struct for serialization
    SerializedSensorData serialized;
    
    // Set magic number for validation
    serialized.magic = 0xABCD1234;
    
    // Copy data fields
    serialized.temp = data.temp;
    serialized.hum = data.hum;
    serialized.pres = data.pres;
    serialized.gasRes = data.gasRes;
    serialized.pm2_5 = data.pm2_5;
    serialized.pm5 = data.pm5;
    serialized.pm10 = data.pm10;
    serialized.padding = 0; // Zero out padding
    serialized.co2 = data.co2;
    serialized.latitude = data.latitude;
    serialized.longitude = data.longitude;
    serialized.timestamp = data.timestamp;
    
    // Set flags
    serialized.flags = data.is_fake_gps ? 0x01 : 0x00;  // Bit 0 = is_fake_gps
    memset(serialized.reserved, 0, sizeof(serialized.reserved));  // Clear reserved bytes
    
    // Zero out the checksum field first to ensure consistent calculation
    serialized.checksum = 0;
    
    // Create a simple numeric checksum from the actual values
    // This is deliberately separate from the binary representation to avoid layout issues
    uint32_t value_checksum = 0xABCD1234;  // Initial seed
    
    // Add the primary numeric values to the checksum
    value_checksum += serialized.co2;
    value_checksum += serialized.pm2_5;
    value_checksum += serialized.pm10;
    value_checksum += serialized.timestamp;
    
    // Add the latitude and longitude
    value_checksum += serialized.latitude % 1000000;  // Only use the less significant digits
    value_checksum += serialized.longitude % 1000000;
    
    // Convert floats to integers in a consistent way
    int temp_int = (int)(serialized.temp * 100);
    int hum_int = (int)(serialized.hum * 100);
    value_checksum += temp_int;
    value_checksum += hum_int;
    
    // Add the flags
    value_checksum += serialized.flags;
    
    // Now set the checksum in the serialized data
    serialized.checksum = value_checksum;
    
    // Debug the binary representation before writing
    printf("SERIALIZED BYTES: ");
    const uint8_t* bytes = (const uint8_t*)&serialized;
    for (size_t i = 0; i < 16; i++) {
        printf("%02x ", bytes[i]);
    }
    printf("...\n");
    
    // Copy the entire struct to the buffer
    memcpy(buffer, &serialized, sizeof(SerializedSensorData));
    
    // Verify the buffer contains what we expect
    printf("BUFFER AFTER MEMCPY: ");
    for (size_t i = 0; i < 16; i++) {
        printf("%02x ", buffer[i]);
    }
    printf("...\n");
}

SensorData Flash::deserializeSensorData(const uint8_t* buffer) {
    // Debug what we're reading from flash
    printf("DESERIALIZING BYTES: ");
    for (size_t i = 0; i < 16; i++) {
        printf("%02x ", buffer[i]);
    }
    printf("...\n");
    
    SensorData data;
    SerializedSensorData serialized;
    
    // Copy from buffer to packed struct
    memcpy(&serialized, buffer, sizeof(SerializedSensorData));
    
    // Debug the binary representation after reading
    printf("SERIALIZED STRUCT BYTES: ");
    const uint8_t* bytes = (const uint8_t*)&serialized;
    for (size_t i = 0; i < 16; i++) {
        printf("%02x ", bytes[i]);
    }
    printf("...\n");
    
    // Validate magic number
    if (serialized.magic != 0xABCD1234) {
        printf("FLASH ERROR: Invalid magic number in record: 0x%08x\n", serialized.magic);
        // Return empty data if validation fails
        return data;
    }
    
    // Save the stored checksum for comparison
    uint32_t stored_checksum = serialized.checksum;
    
    // Zero out the checksum for calculation just like in serialization
    serialized.checksum = 0;
    
    // Calculate the checksum using the exact same method as in serialization
    uint32_t value_checksum = 0xABCD1234;  // Initial seed
    
    // Add the primary numeric values to the checksum
    value_checksum += serialized.co2;
    value_checksum += serialized.pm2_5;
    value_checksum += serialized.pm10;
    value_checksum += serialized.timestamp;
    
    // Add the latitude and longitude
    value_checksum += serialized.latitude % 1000000;  // Only use the less significant digits
    value_checksum += serialized.longitude % 1000000;
    
    // Convert floats to integers in a consistent way
    int temp_int = (int)(serialized.temp * 100);
    int hum_int = (int)(serialized.hum * 100);
    value_checksum += temp_int;
    value_checksum += hum_int;
    
    // Add the flags
    value_checksum += serialized.flags;
    
    // Restore the original checksum for logging
    serialized.checksum = stored_checksum;
    
    // Now the calculated checksum should match what was stored
    if (value_checksum != stored_checksum) {
        printf("ERROR: Checksum mismatch: Expected 0x%08x, Got 0x%08x\n", 
               stored_checksum, value_checksum);
        
        // Instead of immediately returning, check if the data still looks reasonable
        // This can help recover partially corrupted records
        bool data_looks_valid = 
            serialized.temp > -50.0f && serialized.temp < 100.0f && 
            serialized.hum >= 0.0f && serialized.hum <= 100.0f && 
            serialized.timestamp > 1600000000 && serialized.timestamp < 2000000000;
            
        if (!data_looks_valid) {
            printf("ERROR: Data validation failed, record appears corrupted\n");
            return data; // Return empty data
        } else {
            printf("WARNING: Checksum mismatch but data appears valid. Attempting recovery...\n");
            // Continue with recovery attempt
        }
    }
    
    // Extract fields
    data.temp = serialized.temp;
    data.hum = serialized.hum;
    data.pres = serialized.pres;
    data.gasRes = serialized.gasRes;
    data.pm2_5 = serialized.pm2_5;
    data.pm5 = serialized.pm5;
    data.pm10 = serialized.pm10;
    data.co2 = serialized.co2;
    data.latitude = serialized.latitude;
    data.longitude = serialized.longitude;
    data.timestamp = serialized.timestamp;
    
    // Extract flags
    data.is_fake_gps = (serialized.flags & 0x01) != 0;  // Bit 0 = is_fake_gps
    
    // Debug what we extracted
    printf("DESERIALIZED: Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u, Timestamp=%u, FakeGPS=%s\n",
           data.temp, data.hum, data.co2, data.pm2_5, data.timestamp,
           data.is_fake_gps ? "true" : "false");
    
    return data;
}

void Flash::dumpRawFlashContents(size_t max_records) {
    printf("Raw flash contents (first %zu records):\n", max_records);
    
    // Dump count
    const uint32_t* count_ptr = (const uint32_t*)flashAddressToXIP(_data_count_address);
    printf("Count value at 0x%08x: %lu\n", (unsigned int)_data_count_address, *count_ptr);
    
    // Limit to maximum records or stored count, whichever is smaller
    size_t records_to_dump = (_stored_data_count < max_records) ? _stored_data_count : max_records;
    
    for (size_t i = 0; i < records_to_dump; i++) {
        uint32_t data_address = _data_start_address + (i * SENSOR_DATA_SIZE);
        const uint8_t* data_ptr = (const uint8_t*)flashAddressToXIP(data_address);
        
        printf("Record %zu at 0x%08x: ", i, (unsigned int)data_address);
        // Dump first 16 bytes in hex
        for (size_t j = 0; j < 16 && j < SENSOR_DATA_SIZE; j++) {
            printf("%02x ", data_ptr[j]);
        }
        printf("\n");
    }
}

bool Flash::resetStorage() {
    printf("FLASH: Completely erasing flash storage area...\n");
    
    if (!_flash_enabled) {
        printf("FLASH: [DISABLED] Skipping storage reset (operating in memory-only mode)\n");
        _stored_data_count = 0;
        return true;
    }
    
    // Ensure flash address is sector-aligned
    uint32_t flash_offset_aligned = _flash_offset;
    if (flash_offset_aligned % FLASH_SECTOR_SIZE != 0) {
        // Round down to nearest sector boundary
        flash_offset_aligned = (flash_offset_aligned / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
        printf("FLASH: Aligning flash offset to sector boundary: 0x%08x -> 0x%08x\n", 
               (unsigned int)_flash_offset, (unsigned int)flash_offset_aligned);
    }
    
    // Calculate total sectors to erase (16 sectors = 64KB)
    uint32_t sectors_to_erase = 16;
    
    // Use our safe erase function that handles errors gracefully
    if (!safeFlashErase(flash_offset_aligned, sectors_to_erase * FLASH_SECTOR_SIZE)) {
        printf("FLASH ERROR: Failed to erase storage during reset\n");
        return false;
    }
    
    // Reset count in memory
    _stored_data_count = 0;
    
    // For count, ensure we're also sector-aligned
    uint32_t count_sector_offset = _data_count_address % FLASH_SECTOR_SIZE;
    uint32_t count_sector_address = _data_count_address - count_sector_offset;
    
    // Prepare a buffer with the entire sector
    uint8_t* sector_buffer = new uint8_t[FLASH_SECTOR_SIZE];
    if (!sector_buffer) {
        printf("FLASH ERROR: Failed to allocate sector buffer for count\n");
        return false;
    }
    
    // Fill with 0xFF (erased state) first
    memset(sector_buffer, 0xFF, FLASH_SECTOR_SIZE);
    
    // Set count = 0 at the appropriate offset in the buffer
    uint32_t count = 0;
    uint32_t count_offset_in_sector = _data_count_address - count_sector_address;
    memcpy(sector_buffer + count_offset_in_sector, &count, DATA_COUNT_SIZE);
    
    // Use our safe program function to write the count in the context of the sector
    if (!safeFlashProgram(count_sector_address, sector_buffer, FLASH_SECTOR_SIZE)) {
        printf("FLASH ERROR: Failed to write count (0) during reset\n");
        delete[] sector_buffer;
        return false;
    }
    
    delete[] sector_buffer;
    
    printf("FLASH: Storage reset complete - all data and count have been erased\n");
    return true;
}

bool Flash::readSensorDataRecord(uint32_t addr, SensorData &data) {
    // Check if the record would cross a page boundary
    uint32_t page_offset = addr % FLASH_PAGE_SIZE;
    if (page_offset + sizeof(SensorData) > FLASH_PAGE_SIZE) {
        // Handle cross-page reading - need to read in two parts
        uint8_t buffer[sizeof(SensorData)];
        
        // Read first part from current page
        uint32_t first_part_size = FLASH_PAGE_SIZE - page_offset;
        memcpy(buffer, (const void*)addr, first_part_size);
        
        // Read second part from next page
        uint32_t next_page_addr = (addr & ~(FLASH_PAGE_SIZE - 1)) + FLASH_PAGE_SIZE;
        memcpy(buffer + first_part_size, (const void*)next_page_addr, sizeof(SensorData) - first_part_size);
        
        // Copy from buffer to data struct
        memcpy(&data, buffer, sizeof(SensorData));
    } else {
        // Standard single-page read
        memcpy(&data, (const void*)addr, sizeof(SensorData));
    }
    
    // Verify record validity (optional, adjust as needed)
    if (data.timestamp == 0xFFFFFFFF || data.co2 == 0xFFFFFFFF) {
        printf("FLASH: Detected potentially corrupted record at 0x%08x\n", addr);
        return false;
    }
    
    return true;
}

// Create an error sensor data record
SensorData Flash::getSensorDataError() {
    SensorData error;
    // Initialize with default values (the structs already has these as defaults, but setting explicitly for clarity)
    error.timestamp = 0;
    error.latitude = 0;
    error.longitude = 0;
    error.temp = 0.0f;
    error.hum = 0.0f;
    error.pres = 0.0f;
    error.gasRes = 0.0f;
    error.co2 = 0;
    error.pm2_5 = 0;
    error.pm5 = 0;
    error.pm10 = 0;
    error.is_fake_gps = false;
    return error;
}

// Safe flash erase operation with additional checks and recovery
bool Flash::safeFlashErase(uint32_t address, size_t size) {
    if (!_flash_enabled) {
        if (_debug_level > 0) {
            printf("FLASH: [DISABLED] Skipping erase at 0x%08x (size %lu)\n", 
                   (unsigned int)address, (unsigned long)size);
        }
        return true; // Pretend it succeeded when disabled
    }
    
    // Check alignment
    if (address % FLASH_SECTOR_SIZE != 0) {
        printf("FLASH ERROR: Erase address 0x%08x is not sector-aligned\n", (unsigned int)address);
        return false;
    }
    
    if (_debug_level > 0) {
        printf("FLASH: Erasing at address 0x%08x (size %lu bytes)\n", 
               (unsigned int)address, (unsigned long)size);
    }
    
    // Save interrupts, do the erase, then restore
    if (_debug_level > 1) printf("FLASH: Disabling interrupts for erase\n");
    uint32_t interrupt_state = save_and_disable_interrupts();
    
    try {
        flash_range_erase(address, size);
    } catch (...) {
        // This shouldn't happen, but just in case
        restore_interrupts(interrupt_state);
        printf("FLASH ERROR: Exception during erase operation!\n");
        return false;
    }
    
    restore_interrupts(interrupt_state);
    if (_debug_level > 1) printf("FLASH: Interrupts restored after erase\n");
    
    // Add a delay to ensure the erase completes
    if (_debug_level > 0) printf("FLASH: Delay (50ms) after erase to ensure completion\n");
    sleep_ms(50);
    
    // Verify the erase worked by checking a few spots in the range
    if (_debug_level > 0) printf("FLASH: Verifying erase operation\n");
    
    // Check beginning, middle, and end of range
    const uint32_t* verify_begin = (const uint32_t*)flashAddressToXIP(address);
    const uint32_t* verify_middle = (const uint32_t*)flashAddressToXIP(address + (size / 2));
    const uint32_t* verify_end = (const uint32_t*)flashAddressToXIP(address + size - 4);
    
    if (*verify_begin != 0xFFFFFFFF || *verify_middle != 0xFFFFFFFF || *verify_end != 0xFFFFFFFF) {
        printf("FLASH ERROR: Erase verification failed!\n");
        printf("  Begin:  0x%08x (expected 0xFFFFFFFF)\n", (unsigned int)*verify_begin);
        printf("  Middle: 0x%08x (expected 0xFFFFFFFF)\n", (unsigned int)*verify_middle);
        printf("  End:    0x%08x (expected 0xFFFFFFFF)\n", (unsigned int)*verify_end);
        return false;
    }
    
    if (_debug_level > 0) printf("FLASH: Erase operation succeeded\n");
    return true;
}

// Safe flash program operation with additional checks
bool Flash::safeFlashProgram(uint32_t address, const uint8_t* data, size_t size) {
    if (!_flash_enabled) {
        if (_debug_level > 0) {
            printf("FLASH: [DISABLED] Skipping program at 0x%08x (size %lu)\n", 
                   (unsigned int)address, (unsigned long)size);
        }
        return true; // Pretend it succeeded when disabled
    }
    
    // Check alignment
    if (address % 4 != 0) {
        printf("FLASH ERROR: Program address 0x%08x is not word-aligned\n", (unsigned int)address);
        return false;
    }
    
    if (_debug_level > 0) {
        printf("FLASH: Programming at address 0x%08x (size %lu bytes)\n", 
               (unsigned int)address, (unsigned long)size);
    }
    
    // Print first few bytes for debugging
    if (_debug_level > 1) {
        printf("FLASH DEBUG: First bytes to program: ");
        for (size_t i = 0; i < 16 && i < size; i++) {
            printf("%02x ", data[i]);
        }
        printf("\n");
    }
    
    // Save interrupts, do the program, then restore
    if (_debug_level > 1) printf("FLASH: Disabling interrupts for program\n");
    uint32_t interrupt_state = save_and_disable_interrupts();
    
    try {
        flash_range_program(address, data, size);
    } catch (...) {
        // This shouldn't happen, but just in case
        restore_interrupts(interrupt_state);
        printf("FLASH ERROR: Exception during program operation!\n");
        return false;
    }
    
    restore_interrupts(interrupt_state);
    if (_debug_level > 1) printf("FLASH: Interrupts restored after program\n");
    
    // Add a delay to ensure the program completes
    if (_debug_level > 0) printf("FLASH: Delay (50ms) after program to ensure completion\n");
    sleep_ms(50);
    
    // Verify the program worked by checking the data
    if (_debug_level > 0) printf("FLASH: Verifying program operation\n");
    
    // Only check the first few bytes for efficiency
    size_t verify_size = std::min(size, (size_t)16);
    const uint8_t* verify_data = (const uint8_t*)flashAddressToXIP(address);
    
    bool verification_passed = true;
    for (size_t i = 0; i < verify_size; i++) {
        if (verify_data[i] != data[i]) {
            printf("FLASH ERROR: Program verification failed at offset %lu!\n", (unsigned long)i);
            printf("  Expected: 0x%02x, Got: 0x%02x\n", data[i], verify_data[i]);
            verification_passed = false;
            break;
        }
    }
    
    if (verification_passed) {
        if (_debug_level > 0) printf("FLASH: Program operation succeeded\n");
        return true;
    } else {
        return false;
    }
}
