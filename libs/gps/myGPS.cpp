//
// Created by Benedikt Walter on 18.01.24.
//

#include "pico/stdlib.h"
#include "stdlib.h"
#include "libs/gps/myGPS.h"
#include <hardware/gpio.h>
#include <time.h>  // Add for time functions
#include <cmath>

myGPS::myGPS(uart_inst_t *uart_id, int baud_rate, int tx_pin, int rx_pin) {
    this->uart_id = uart_id;
    this->baud_rate = baud_rate;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->init();
}

void myGPS::init() {
    // Initialize UART with higher priority settings
    uart_init(this->uart_id, this->baud_rate);
    
    // Configure UART pins
    gpio_set_function(this->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(this->rx_pin, GPIO_FUNC_UART);
    
    // Enable pull-up on RX pin to improve signal integrity
    gpio_pull_up(this->rx_pin);
    
    // Set UART FIFO thresholds for better responsiveness
    // Enable FIFO and set receive interrupt trigger at 1/8 full (more responsive)
    uart_set_fifo_enabled(this->uart_id, true);
    
    // Flush any pending data in the UART
    while (uart_is_readable(this->uart_id)) {
        uart_getc(this->uart_id);
    }
    
    printf("GPS UART initialized with optimized settings\n");
}

/** /@return 0 on sucess \n 1 on not sucess \n 2 on invalid fix
 */
int myGPS::readLine(std::string &line) {
    // If fake GPS mode is enabled, generate data immediately
    if (use_fake_data) {
        // Generate a fake NMEA sentence for GLL format with current timestamp
        char fake_buffer[256];
        
        // Get current time for the fake data
        time_t timestamp = ::time(NULL);
        struct tm *timeinfo = gmtime(&timestamp);
        
        // Format the time as HHMMSS.sss
        char time_str[15];
        sprintf(time_str, "%02d%02d%02d.000", 
                timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
        
        // Add small random variations to simulate GPS accuracy fluctuations
        // Use time as seed for randomization to get different values each second
        srand(timestamp);
        double random_lat_offset = ((rand() % 100) - 50) * 0.000005;
        double random_lon_offset = ((rand() % 100) - 50) * 0.000005;
        
        double lat_with_noise = fake_latitude + random_lat_offset;
        double lon_with_noise = fake_longitude + random_lon_offset;
        
        // Format coordinates in NMEA format
        double lat_degrees = std::floor(lat_with_noise);
        double lat_minutes = (lat_with_noise - lat_degrees) * 60.0;
        double lon_degrees = std::floor(lon_with_noise);
        double lon_minutes = (lon_with_noise - lon_degrees) * 60.0;
        
        // Format the NMEA sentence
        sprintf(fake_buffer, "$GNGLL,%02.0f%07.4f,%c,%03.0f%07.4f,%c,%s,A,*XX\r\n",
                lat_degrees, lat_minutes, 
                lat_with_noise >= 0 ? 'N' : 'S',
                lon_degrees, lon_minutes, 
                lon_with_noise >= 0 ? 'E' : 'W',
                time_str);
        
        line = fake_buffer;
        
        // Update internal variables
        this->latitude = lat_with_noise;
        this->longitude = lon_with_noise;
        this->nsIndicator = lat_with_noise >= 0 ? 'N' : 'S';
        this->ewIndicator = lon_with_noise >= 0 ? 'E' : 'W';
        this->time = time_str;
        
        // Format the date for internal use
        char date_buffer[7];
        sprintf(date_buffer, "%02d%02d%02d", 
                timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year % 100);
        this->date = date_buffer;
        
        // Static time-limited debug output (1/10th of fake positions)
        static uint32_t last_debug_time = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (now - last_debug_time > 10000) { // Every 10 seconds
            printf("FAKE GPS: Position: %.6f,%c %.6f,%c (simulated)\n", 
                  lat_with_noise, lat_with_noise >= 0 ? 'N' : 'S',
                  lon_with_noise, lon_with_noise >= 0 ? 'E' : 'W');
            last_debug_time = now;
        }
        
        // Return 0 to indicate valid fix
        return 0;
    }

    // For real GPS data, use even shorter timeouts for quicker polling
    absolute_time_t master_timeout = make_timeout_time_ms(200); // Reduced from 300ms to 200ms
    
    // Check if UART has data - if not, return quickly
    if(!uart_is_readable(this->uart_id)) {
        return 1; // No data available yet
    }

    // Use an even shorter timeout for each polling cycle
    absolute_time_t timeout_time = make_timeout_time_ms(100); // Reduced from 200ms to 100ms
    
    bool valid_sentence_found = false;
    std::string sentence_type;
    
    // Safety counter with reduced maximum for faster returns
    int loop_counter = 0;
    const int MAX_LOOP_ITERATIONS = 100; // Reduced from 200 to 100 for faster returns
    
    // Single NMEA sentence buffer with pre-allocated capacity
    this->buffer.reserve(100); // Pre-allocate memory to reduce reallocations
    this->buffer.clear();      // Clear buffer for new data
    
    // Keep trying until we find a valid sentence or timeout
    while (!valid_sentence_found && 
           !absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0 &&
           loop_counter++ < MAX_LOOP_ITERATIONS) {
        
        // If buffer isn't empty and doesn't end with newline, continue reading
        while(this->buffer.empty() || this->buffer.back() != '\n') {
            // Break on timeout
            if (absolute_time_diff_us(get_absolute_time(), master_timeout) <= 0) {
                return 1; // Timeout reading GPS
            }
            
            if(uart_is_readable(this->uart_id)) {
                char c = uart_getc(this->uart_id);
                this->buffer += c;
                
                // Look for end of sentence
                if (c == '\n') {
                    break;
                }
            } else {
                sleep_us(100); // Reduced from 1ms to 100μs for faster polling
            }
            
            // Buffer safety check
            if(this->buffer.length() > 100) {
                // Buffer overflow, reset
                this->buffer.clear();
                break;
            }
        }
        
        if(buffer.empty()) {
            continue;
        }
        
        // Only print raw GPS NMEA sentences if specifically debugging GPS
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("%s", this->buffer.c_str());
#endif
        
        // Quickly check for sentence type
        if (this->buffer.find(GNGLL) == 0 || this->buffer.find("$GPGLL") == 0) {
            valid_sentence_found = true;
            sentence_type = "GLL";
        } else if (this->buffer.find(GNRMC) == 0 || this->buffer.find("$GPRMC") == 0) {
            valid_sentence_found = true;
            sentence_type = "RMC";
        }
    }
    
    // If we didn't find a valid sentence
    if (!valid_sentence_found) {
        return 1; // No valid data
    }

    // Return the found sentence
    line = this->buffer;
    
    // Now parse the sentence to extract coordinates and fix status
    std::istringstream iss(this->buffer);
    std::string token;

    // Skip the first token (sentence identifier)
    std::getline(iss, token, ',');
    
    // Faster parsing with direct string operations rather than multiple getlines
    if (sentence_type == "GLL") {
        // Parse GLL sentence
        std::getline(iss, token, ','); // Latitude
        if (!token.empty() && token.length() >= 3) {
            try {
                this->latitude = std::stod(token.substr(0, 2)) + std::stod(token.substr(2)) / 60.0;
            } catch (...) {
                this->latitude = 0;
            }
        }

        std::getline(iss, token, ','); // N/S indicator
        if (!token.empty()) this->nsIndicator = token[0];

        std::getline(iss, token, ','); // Longitude
        if (!token.empty() && token.length() >= 4) {
            try {
                this->longitude = std::stod(token.substr(0, 3)) + std::stod(token.substr(3)) / 60.0;
            } catch (...) {
                this->longitude = 0;
            }
        }

        std::getline(iss, token, ','); // E/W indicator
        if (!token.empty()) this->ewIndicator = token[0];

        std::getline(iss, token, ','); // Time
        if (!token.empty() && token.size() >= 6) {
            this->time = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
        }

        std::getline(iss, token, ','); // Fix validity (A=valid, V=invalid)
        bool valid_fix = (token == "A");
        
        return valid_fix ? 0 : 2;  // Return 0 for valid fix, 2 for invalid
    }
    else if (sentence_type == "RMC") {
        // Parse RMC sentence - more efficient parsing
        std::getline(iss, token, ','); // Time
        if (!token.empty() && token.size() >= 6) {
            this->time = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
        }
        
        std::getline(iss, token, ','); // Fix validity
        bool valid_fix = (token == "A");
        
        if (!valid_fix) return 2; // Early return if invalid fix
        
        std::getline(iss, token, ','); // Latitude
        if (!token.empty() && token.length() >= 3) {
            try {
                this->latitude = std::stod(token.substr(0, 2)) + std::stod(token.substr(2)) / 60.0;
            } catch (...) {
                this->latitude = 0;
            }
        }
        
        std::getline(iss, token, ','); // N/S indicator
        if (!token.empty()) this->nsIndicator = token[0];
        
        std::getline(iss, token, ','); // Longitude
        if (!token.empty() && token.length() >= 4) {
            try {
                this->longitude = std::stod(token.substr(0, 3)) + std::stod(token.substr(3)) / 60.0;
            } catch (...) {
                this->longitude = 0;
            }
        }
        
        std::getline(iss, token, ','); // E/W indicator
        if (!token.empty()) this->ewIndicator = token[0];
        
        // Skip speed and course
        std::getline(iss, token, ',');
        std::getline(iss, token, ',');
        
        // Date field (format: ddmmyy)
        std::getline(iss, token, ',');
        if (!token.empty() && token.size() >= 6) {
            this->date = token;
        }
        
        return valid_fix ? 0 : 2;  // Return 0 for valid fix, 2 for invalid
    }
    
    // If we get here, something went wrong with parsing
    return 1;
}

int myGPS::readLine(std::string &buffer, double &longitude, char &ewIndicator, double &latitude, char &nsIndicator, std::string &time) {
    // If fake GPS data is enabled, return simulated data
    if (use_fake_data) {
        // Generate fake GPS data
        buffer = "$GNGLL,4812.3972,N,1537.0508,E,120000.000,A,*XX";
        longitude = fake_longitude;
        latitude = fake_latitude;
        ewIndicator = (fake_longitude >= 0) ? 'E' : 'W';
        nsIndicator = (fake_latitude >= 0) ? 'N' : 'S';
        
        // Get current time for the fake data
        time_t gps_timestamp = ::time(NULL);
        struct tm *timeinfo = gmtime(&gps_timestamp);
        char time_buffer[9];
        strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", timeinfo);
        time = time_buffer;
        
        // Return 0 to indicate successful read with valid fix
        return 0;
    }
    
    // If not using fake data, use the original implementation
    return readLine(buffer);
}

int myGPS::readLine(std::string &buffer, double &longitude, char &ewIndicator, double &latitude, char &nsIndicator, std::string &time, std::string &date) {
    if (use_fake_data) {
        // Generate current timestamp in HHMMSS format
        time_t timestamp = ::time(NULL);
        struct tm *timeinfo = gmtime(&timestamp);
        char time_buffer[7];
        sprintf(time_buffer, "%02d%02d%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
        time = time_buffer;
        
        // Format date in DDMMYY format
        char date_buffer[7];
        sprintf(date_buffer, "%02d%02d%02d", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year % 100);
        date = date_buffer;
        
        // Add small random variations to simulate GPS accuracy fluctuations
        // Use time as seed for randomization to get different values each second
        srand(timestamp);
        double random_lat_offset = ((rand() % 100) - 50) * 0.000005;
        double random_lon_offset = ((rand() % 100) - 50) * 0.000005;
        
        latitude = fake_latitude + random_lat_offset;
        longitude = fake_longitude + random_lon_offset;
        
        // Set indicators (North/East for Vienna area)
        nsIndicator = (latitude >= 0) ? 'N' : 'S';
        ewIndicator = (longitude >= 0) ? 'E' : 'W';
        
        // Simulate a valid NMEA sentence
        char nmea_buffer[256];
        sprintf(nmea_buffer, "$GNRMC,%s.000,A,%02d%07.4f,%c,%03d%07.4f,%c,0.00,0.00,%s,,,A",
                time.c_str(),
                (int)latitude, (latitude - (int)latitude) * 60.0, nsIndicator,
                (int)longitude, (longitude - (int)longitude) * 60.0, ewIndicator,
                date.c_str());
        
        buffer = nmea_buffer;
        
        // Debug info about fake GPS data - using a timer to reduce spam
        static uint32_t last_print_time = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (now - last_print_time > 5000) { // Print only every 5 seconds
            printf("FAKE GPS: Position: %f,%c %f,%c (random variation)\n", 
                  latitude, nsIndicator, longitude, ewIndicator);
            last_print_time = now;
        }
        
        return 0; // Success
    }
    
    // For real GPS, use a strict timeout to prevent blocking
    // Reading with all parameters to get the data
    bool use_safe_values = true;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    // Call the simpler readLine method first - with 1 second max timeout
    int result = readLine(buffer);
    
    // Calculate time spent in GPS read
    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
    if (elapsed > 500) {
        printf("WARNING: GPS read took %lu ms (expected <500ms)\n", elapsed);
    }
    
    if (result == 0) {
        // If we got good data, use it
        use_safe_values = false;
    } else {
        // If real read failed, use last known values for safety
        printf("GPS read failed or timed out, using last known values\n");
    }
    
    // Return longitude, latitude, indicators and time from class members
    longitude = this->longitude;
    ewIndicator = this->ewIndicator;
    latitude = this->latitude;
    nsIndicator = this->nsIndicator;
    time = this->time;
    date = this->date;
    
    return result;
}

std::string myGPS::to_string(double latitude, char nsIndicator, double longitude, char ewIndicator, std::string &time) {
    std::string flash_data;
    for(auto i : std::to_string(latitude)) {
        flash_data.push_back(i);
    }
    flash_data.push_back('|');
    flash_data.push_back(nsIndicator);
    flash_data.push_back('|');
    for(auto i : std::to_string(longitude)) {
        flash_data.push_back(i);
    }
    flash_data.push_back('|');
    flash_data.push_back(ewIndicator);
    flash_data.push_back('|');
    for(auto i : time) {
        flash_data.push_back(i);
    }
    flash_data.push_back('^');
    return flash_data;
}

int myGPS::testConnection() {
    if (use_fake_data) {
        // If in fake mode, always return good connection status
        printf("FAKE GPS: Connection test - simulating good connection\n");
        return 0; // Good connection with valid NMEA data
    }
    
    // Original implementation continues below
    // Attempt to read from the module with a timeout
    printf("Testing GPS connection...\n");
    
    // First, check if UART is configured properly - don't use uart_is_enabled
    // Use a simpler approach by trying to reinitialize UART
    printf("Reinitializing UART to ensure it's enabled...\n");
    this->init();
    sleep_ms(100); // Give it time to initialize
    
    // Attempt to read data (any data) from UART for 3 seconds
    printf("Checking if GPS module is sending any data...\n");
    absolute_time_t timeout = make_timeout_time_ms(3000);
    bool any_data_received = false;
    bool valid_nmea_seen = false;
    bool frame_errors_seen = false;
    int chars_received = 0;
    std::string sample_data;
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
        if (uart_is_readable(this->uart_id)) {
            char c = uart_getc(this->uart_id);
            chars_received++;
            
            // Keep a small sample of the data for debugging
            if (sample_data.length() < 100) {
                sample_data += c;
            }
            
            // Look for NMEA sentence start character '$'
            if (c == '$') {
                any_data_received = true;
                
                // Try to read a complete NMEA sentence
                std::string nmea_sentence;
                nmea_sentence += c;
                
                absolute_time_t sentence_timeout = make_timeout_time_ms(500);
                bool complete_sentence = false;
                
                // Read until newline or timeout
                while (!complete_sentence && !absolute_time_diff_us(get_absolute_time(), sentence_timeout) <= 0) {
                    if (uart_is_readable(this->uart_id)) {
                        char nc = uart_getc(this->uart_id);
                        chars_received++;
                        nmea_sentence += nc;
                        
                        // Look for end of NMEA sentence
                        if (nc == '\n' || nc == '\r') {
                            complete_sentence = true;
                        }
                    } else {
                        sleep_ms(1);
                    }
                    
                    // Check for excessively long sentence (likely corrupt data)
                    if (nmea_sentence.length() > 100) {
                        break;
                    }
                }
                
                // Check if we found a valid NMEA sentence
                if (complete_sentence && nmea_sentence.length() > 6) {
                    // Check if it contains "More than 100 frame errors"
                    if (nmea_sentence.find("frame errors") != std::string::npos) {
                        frame_errors_seen = true;
                        printf("Frame errors detected in GPS data\n");
                    } else {
                        valid_nmea_seen = true;
                        printf("Valid NMEA sentence received: %s", nmea_sentence.c_str());
                    }
                }
            }
            
            any_data_received = true;
        } else {
            sleep_ms(10);
        }
        
        // Exit early if we've already seen valid NMEA data
        if (valid_nmea_seen && chars_received > 100) {
            break;
        }
    }
    
    printf("GPS test complete. Received %d characters.\n", chars_received);
    
    if (!any_data_received) {
        printf("No data received from GPS module. Check connections and power.\n");
        return 2; // UART communication error
    }
    
    if (frame_errors_seen) {
        printf("Frame errors detected. Likely baud rate mismatch or noisy connection.\n");
        printf("Try different baud rates: 4800, 9600, 38400, etc.\n");
        printf("Current baud rate: %d\n", this->baud_rate);
        return 3; // Frame errors
    }
    
    if (!valid_nmea_seen) {
        if (chars_received > 0 && chars_received < 10) {
            printf("Minimal data received. Likely incorrect baud rate.\n");
            printf("Sample data received: %s\n", sample_data.c_str());
            return 4; // Likely baud rate issue
        } else {
            printf("Data received but no valid NMEA sentences. GPS module may be starting up.\n");
            printf("Sample data received: %s\n", sample_data.c_str());
            return 1; // No valid NMEA data
        }
    }
    
    printf("GPS connection test successful. Valid NMEA data received.\n");
    return 0; // Good connection with valid data
}

int myGPS::getVisibleSatellites() {
    if (use_fake_data) {
        // If in fake GPS mode, first simulate gradual satellite acquisition
        if (fake_startup_time == 0) {
            // First call - start acquisition simulation
            fake_startup_time = to_ms_since_boot(get_absolute_time());
            fake_satellites = 0;
            fake_fix_acquired = false;
            return 0;
        }
        
        // Calculate elapsed time since startup
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - fake_startup_time;
        
        // Simulate finding satellites gradually over 15 seconds
        if (elapsed < fake_acquisition_time_ms) {
            // Gradually increase satellites from 0 to 7 during acquisition
            fake_satellites = (7 * elapsed) / fake_acquisition_time_ms;
            // Sometimes show one less to simulate fluctuation
            if (rand() % 10 == 0) {
                fake_satellites = fake_satellites > 0 ? fake_satellites - 1 : 0;
            }
        } else if (!fake_fix_acquired) {
            // After acquisition time, we have a fix with 7-10 satellites
            fake_fix_acquired = true;
            fake_satellites = 7 + (rand() % 4);
            printf("FAKE GPS: Fix acquired with %d satellites after %d ms\n", 
                   fake_satellites, elapsed);
        } else {
            // After fix, occasionally vary satellite count to simulate real-world changes
            if (rand() % 20 == 0) {
                int change = (rand() % 3) - 1; // -1, 0, or +1
                fake_satellites += change;
                // Keep within reasonable range
                if (fake_satellites < 6) fake_satellites = 6;
                if (fake_satellites > 12) fake_satellites = 12;
            }
        }
        
        return fake_satellites;
    }
    
    // Original implementation continues below for real GPS
    // Attempt to read from the GPS module for up to 2 seconds
    absolute_time_t timeout = make_timeout_time_ms(2000);
    int satellite_count = 0;
    bool found_gsv_message = false;
    
    printf("Checking for visible satellites...\n");
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0 && !found_gsv_message) {
        if (uart_is_readable(this->uart_id)) {
            std::string gsv_message = "";
            char c = uart_getc(this->uart_id);
            
            // Look for the start of a GSV message
            if (c == '$') {
                gsv_message += c;
                
                // Read more characters to check if this is a GSV message
                for (int i = 0; i < 5; i++) {
                    if (uart_is_readable(this->uart_id)) {
                        c = uart_getc(this->uart_id);
                        gsv_message += c;
                    } else {
                        sleep_ms(1);
                    }
                }
                
                // If we've found a GSV message (satellites in view)
                if (gsv_message == "$GPGSV" || gsv_message == "$GNGSV" || gsv_message == "$GLGSV") {
                    found_gsv_message = true;
                    
                    // Continue reading the rest of the message
                    bool reading_message = true;
                    while (reading_message) {
                        if (uart_is_readable(this->uart_id)) {
                            c = uart_getc(this->uart_id);
                            gsv_message += c;
                            
                            // End of message
                            if (c == '\n' || c == '\r') {
                                reading_message = false;
                            }
                        } else {
                            sleep_ms(1);
                        }
                        
                        // Safety check for message length
                        if (gsv_message.length() > 100) {
                            reading_message = false;
                        }
                    }
                    
                    // Parse the GSV message to get satellite count
                    std::istringstream iss(gsv_message);
                    std::string token;
                    
                    // GSV message format: $GPGSV,3,1,11,...
                    // Third field is the total number of satellites in view
                    
                    // Skip the header
                    std::getline(iss, token, ',');
                    
                    // Skip the total message count
                    std::getline(iss, token, ',');
                    
                    // Skip the message number
                    std::getline(iss, token, ',');
                    
                    // Get the number of satellites in view
                    std::getline(iss, token, ',');
                    if (!token.empty()) {
                        try {
                            satellite_count = std::stoi(token);
                            printf("Satellites in view: %d\n", satellite_count);
                        } catch (const std::exception& e) {
                            printf("Error parsing satellite count: %s\n", e.what());
                        }
                    }
                }
            }
        } else {
            sleep_ms(10);
        }
    }
    
    return satellite_count;
}

// Add a helper method to assist with getting a position fix
bool myGPS::waitForFix(int timeout_seconds) {
    // If using fake GPS, simulate the acquisition process
    if (use_fake_data) {
        // Reset simulation state if this is the first call after enabling fake mode
        if (fake_startup_time == 0) {
            printf("FAKE GPS: Starting acquisition simulation\n");
            fake_startup_time = to_ms_since_boot(get_absolute_time());
            fake_satellites = 0;
            fake_fix_acquired = false;
            return false; // First call always returns no fix
        }
        
        // Get the elapsed time since we started the fake GPS simulation
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - fake_startup_time;
        
        // Simulate a realistic acquisition time - typical cold start takes 5-30 seconds
        if (elapsed < fake_acquisition_time_ms) {
            // Gradually increase satellites during acquisition
            fake_satellites = (7 * elapsed) / fake_acquisition_time_ms;
            
            // Log progress but not too frequently
            static uint32_t last_log_time = 0;
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_log_time > 1000) { // Log once per second
                printf("FAKE GPS: Acquiring satellites... %d found (%d ms elapsed)\n", 
                       fake_satellites, elapsed);
                last_log_time = now;
            }
            
            // No fix yet during acquisition phase
            return false;
        } else {
            // After acquisition time, we have a fix
            if (!fake_fix_acquired) {
                printf("FAKE GPS: Fix acquired after %d ms\n", elapsed);
                fake_fix_acquired = true;
                fake_satellites = 7 + (rand() % 4); // 7-10 satellites when fixed
            }
            return true; // We have a fix
        }
    }
    
    // Real GPS implementation starts here
    printf("Waiting for GPS fix (timeout: %d seconds)...\n", timeout_seconds);
    
    absolute_time_t timeout = make_timeout_time_ms(timeout_seconds * 1000);
    std::string buffer;
    bool got_fix = false;
    
    // Reset receiver if we haven't received any valid data
    if (!uart_is_readable(this->uart_id)) {
        printf("No data from GPS, reinitializing...\n");
        this->init();
        sleep_ms(200);
    }
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0 && !got_fix) {
        // Check for readable data
        if (uart_is_readable(this->uart_id)) {
            try {
                // Try to read a full line with fix information
                std::string tmp_buffer;
                double longitude = 0, latitude = 0;
                char ewIndicator = ' ', nsIndicator = ' ';
                std::string time;
                
                int status = this->readLine(tmp_buffer, longitude, ewIndicator, latitude, nsIndicator, time);
                
                // If we got a valid fix, break out of the loop
                if (status == 0) {
                    buffer = tmp_buffer;
                    got_fix = true;
                    printf("Got GPS fix! Lat: %f%c, Long: %f%c\n", 
                           latitude, nsIndicator, longitude, ewIndicator);
                } else {
                    // If we're still waiting, check every 5 seconds how many satellites we can see
                    static uint32_t last_sat_check = 0;
                    uint32_t now = to_ms_since_boot(get_absolute_time());
                    
                    if (now - last_sat_check > 5000) {
                        int sats = this->getVisibleSatellites();
                        printf("Waiting for fix... Satellites in view: %d\n", sats);
                        last_sat_check = now;
                        
                        // Calculate remaining time
                        int remaining = (timeout_seconds * 1000 - (now - to_ms_since_boot(timeout) + timeout_seconds * 1000)) / 1000;
                        printf("Timeout in %d seconds\n", remaining > 0 ? remaining : 0);
                    }
                    
                    // Small delay to not flood the terminal
                    sleep_ms(200);
                }
            } catch (const std::exception& e) {
                printf("Exception while waiting for fix: %s\n", e.what());
                sleep_ms(100);
            }
        } else {
            sleep_ms(100);
        }
    }
    
    if (!got_fix) {
        printf("Timeout waiting for GPS fix\n");
    }
    
    return got_fix;
}

// Send hot start command to the GPS module
bool myGPS::sendHotStartCommand() {
    printf("Sending GPS hot start command...\n");
    
    // Ensure UART is initialized - don't use uart_is_enabled
    // Just reinitialize to be safe
    this->init();
    sleep_ms(100);
    
    // For NMEA GPS modules, the hot start command varies by manufacturer
    // This is the most common format, compatible with many GPS modules
    // $PMTK101*32<CR><LF> - Hot start (MTK chipset)
    const char *hot_start_cmd = "$PMTK101*32\r\n";
    
    // Send the command
    for (const char *p = hot_start_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    // Add a small delay to let the command take effect
    sleep_ms(500);
    
    // Also send specific commands to enable time output
    // $PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28 - Enable RMC and GLL sentences
    const char *enable_time_cmd = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    
    for (const char *p = enable_time_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(200);
    
    // Also try u-blox format (used in many GPS modules)
    // Warm start for u-blox
    const char *ublox_warm_start = "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n"; // Enable GLL messages
    for (const char *p = ublox_warm_start; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(200);
    
    // Also enable RMC messages which contain time info
    const char *ublox_enable_rmc = "$PUBX,40,RMC,0,1,0,0,0,0*47\r\n";
    for (const char *p = ublox_enable_rmc; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(500);
    
    // Check if we're receiving data after the restart
    int attempts = 10;
    bool data_received = false;
    
    while (attempts > 0 && !data_received) {
        sleep_ms(100);
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
        }
        attempts--;
    }
    
    if (data_received) {
        printf("GPS module responded after hot start command\n");
        
        // Flush any pending data
        while (uart_is_readable(this->uart_id)) {
            uart_getc(this->uart_id);
        }
        
        return true;
    } else {
        printf("No response from GPS module after hot start command\n");
        return false;
    }
}

// Send warm start command to the GPS module
bool myGPS::sendWarmStartCommand() {
    printf("Sending GPS warm start command...\n");
    
    // Ensure UART is initialized - don't use uart_is_enabled
    // Just reinitialize to be safe
    this->init();
    sleep_ms(100);
    
    // For NMEA GPS modules, the warm start command varies by manufacturer
    // This is the most common format for MTK chipsets
    // $PMTK102*31<CR><LF> - Warm start
    const char *warm_start_cmd = "$PMTK102*31\r\n";
    
    // Send the command
    for (const char *p = warm_start_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    // Add a small delay to let the command take effect
    sleep_ms(500);
    
    // Check if we're receiving data after the restart
    int attempts = 10;
    bool data_received = false;
    
    while (attempts > 0 && !data_received) {
        sleep_ms(100);
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
        }
        attempts--;
    }
    
    if (data_received) {
        printf("GPS module responded after warm start command\n");
        
        // Flush any pending data
        while (uart_is_readable(this->uart_id)) {
            uart_getc(this->uart_id);
        }
        
        return true;
    } else {
        printf("No response from GPS module after warm start command\n");
        return false;
    }
}

// Send a command to enable time messages from the GPS module
bool myGPS::enableTimeMessages() {
    printf("Sending command to enable GPS time messages...\n");
    
    // Ensure UART is initialized - don't use uart_is_enabled
    // Just reinitialize to be safe
    this->init();
    sleep_ms(100);
    
    // Try multiple command formats to cover different GPS module types
    
    // 1. For MTK chipsets - Enable GLL messages which contain time
    const char *mtk_enable_gll = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    
    // 2. For u-blox modules - Enable GLL messages
    const char *ublox_enable_gll = "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n";
    
    // 3. For SiRF modules - Enable NMEA output
    const char *sirf_enable_nmea = "$PSRF100,1,9600,8,1,0*0C\r\n";
    
    // Send all commands with delays between them
    printf("Sending MTK command...\n");
    for (const char *p = mtk_enable_gll; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(200);
    
    printf("Sending u-blox command...\n");
    for (const char *p = ublox_enable_gll; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(200);
    
    printf("Sending SiRF command...\n");
    for (const char *p = sirf_enable_nmea; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(500);
    
    // Flush any pending data
    while (uart_is_readable(this->uart_id)) {
        uart_getc(this->uart_id);
    }
    
    printf("Time message commands sent, waiting for response...\n");
    
    // Check if we're getting any data back
    absolute_time_t timeout = make_timeout_time_ms(3000);
    bool got_response = false;
    std::string received_data;
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0 && received_data.length() < 100) {
        if (uart_is_readable(this->uart_id)) {
            char c = uart_getc(this->uart_id);
            received_data += c;
            got_response = true;
            
            // If we've received a full line, break
            if (c == '\n' && received_data.find('$') != std::string::npos) {
                break;
            }
        } else {
            sleep_ms(10);
        }
    }
    
    if (got_response) {
        printf("Received response from GPS module: %s\n", received_data.c_str());
        return true;
    } else {
        printf("No response from GPS module after time message commands\n");
        return false;
    }
}

// Send cold start command to the GPS module
bool myGPS::sendColdStartCommand() {
    printf("Sending GPS cold start command...\n");
    
    // Ensure UART is initialized - don't use uart_is_enabled
    // Just reinitialize to be safe
    this->init();
    sleep_ms(100);
    
    // For NMEA GPS modules, the cold start command varies by manufacturer
    // This is the most common format for MTK chipsets
    // $PMTK103*30<CR><LF> - Cold start
    const char *cold_start_cmd = "$PMTK103*30\r\n";
    
    // Send the command
    for (const char *p = cold_start_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    // Add a longer delay for cold start since it's a full reset
    sleep_ms(1000);
    
    // Also try u-blox format for cold start
    // $PUBX,40,GLL,0,1,0,0,0,0*5D\r\n - Enable GLL messages after reset
    const char *ublox_cold_start = "$PUBX,104*37\r\n";
    
    // Send the command
    for (const char *p = ublox_cold_start; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(1000);
    
    // Configure NMEA sentences after cold start
    const char *enable_sentences = "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    
    // Send the command to enable specific NMEA sentences
    for (const char *p = enable_sentences; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(500);
    
    // Check if we're receiving data after the cold start
    int attempts = 20; // More attempts for cold start
    bool data_received = false;
    
    printf("Waiting for GPS to restart after cold start...\n");
    
    while (attempts > 0 && !data_received) {
        sleep_ms(200);
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
        }
        attempts--;
    }
    
    if (data_received) {
        printf("GPS module responded after cold start command\n");
        
        // Flush any pending data
        while (uart_is_readable(this->uart_id)) {
            uart_getc(this->uart_id);
        }
        
        // Send enable time messages command after cold start
        enableTimeMessages();
        
        return true;
    } else {
        printf("No response from GPS module after cold start command\n");
        return false;
    }
}

// Add a new function to optimize GPS for faster fix acquisition (add after sendColdStartCommand)
bool myGPS::optimizeForFastAcquisition() {
    printf("Optimizing GPS for faster fix acquisition...\n");
    
    // Ensure UART is initialized
    this->init();
    sleep_ms(100);
    
    // For NMEA GPS modules, try various commands to optimize for faster acquisition
    // These may not all work on every GPS module but should not cause harm
    
    // 1. Enable all NMEA sentences to maximize data for processing
    // PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28 - Enable all position sentences
    const char *enable_all = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    for (const char *p = enable_all; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(100);
    
    // 2. Set update rate to 1Hz for better sensitivity (MTK chipsets)
    // PMTK220,1000 - Update position every 1000ms
    const char *update_rate = "$PMTK220,1000*1F\r\n";
    for (const char *p = update_rate; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(100);
    
    // 3. Enable SBAS (Satellite-Based Augmentation System) if available
    // PMTK313,1 - Enable SBAS
    const char *enable_sbas = "$PMTK313,1*2E\r\n";
    for (const char *p = enable_sbas; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(100);
    
    // 4. Set position fix interval (MTK chipsets)
    // PMTK300,1000,0,0,0,0 - Try to get position fix every 1000ms
    const char *fix_interval = "$PMTK300,1000,0,0,0,0*1C\r\n";
    for (const char *p = fix_interval; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(100);
    
    // 5. Try u-blox specific commands for UART configuration
    // This is for u-blox modules - configure NMEA protocol
    const char *ublox_nmea = "$PUBX,41,1,0007,0003,9600,0*10\r\n";
    for (const char *p = ublox_nmea; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(100);
    
    printf("GPS optimization commands sent\n");
    
    // Verify we're still getting data from the GPS
    int timeout_ms = 2000; // 2 second timeout
    absolute_time_t start_time = get_absolute_time();
    bool data_received = false;
    
    while (!data_received && !absolute_time_diff_us(get_absolute_time(), start_time) <= 0) {
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
            
            // Drain the buffer to avoid processing stale data
            while (uart_is_readable(this->uart_id)) {
                uart_getc(this->uart_id);
                sleep_ms(1);
            }
        }
        sleep_ms(10);
    }
    
    if (data_received) {
        printf("GPS module is responding after optimization\n");
        return true;
    } else {
        printf("WARNING: No response from GPS after optimization commands\n");
        return false;
    }
}

