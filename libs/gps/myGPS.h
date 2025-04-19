//
// Created by Benedikt Walter on 18.01.24.
//

#ifndef MY_PROJECT_MYGPS_H
#define MY_PROJECT_MYGPS_H


#include "hardware/uart.h"
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#ifdef ERROR_GPS_LOG
#define ERROR_GPS(fmt, ...) printf("ERROR-GPS: " fmt "\n", ##__VA_ARGS__)
#else
#define ERROR_GPS(fmt, ...)
#endif

#ifdef DEBUG_GPS_LOG
#define DEBUG_GPS(fmt, ...) printf("DEBUG-GPS: " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG_GPS(fmt, ...)
#endif



#define UART0_UART_ID uart0
#define UART0_BAUD_RATE 9600
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

const std::string GNTXT = "$GNTXT";
const std::string GNGLL = "$GNGLL";
const std::string GNRMC = "$GNRMC";

const std::string AUTHREQ = "$AUTHREQ";
const std::string AUTHRES = "$AUTHRES";
const std::string DATASEND = "$DATASEND";
const std::string DATAACKN = "$DATAACKN";

class myGPS {
private:
    uart_inst_t *uart_id;
    int baud_rate;
    int tx_pin;
    int rx_pin;
    double latitude = 0;
    char nsIndicator = 'C';
    double longitude = 0;
    char ewIndicator = 'C';
    std::string time = "00:00:00";
    std::string date = "010170"; // Default date (January 1, 1970) in ddmmyy format
    std::string buffer;
    
    // Fake GPS data flag and simulated coordinates
    bool use_fake_data = false;
    double fake_latitude = 48.20662016908546;    // Default fake latitude
    double fake_longitude = 15.617513602109687;  // Default fake longitude
    
    // For simulating GPS acquisition
    uint32_t fake_startup_time = 0;
    bool fake_fix_acquired = false;
    int fake_satellites = 0;
    int fake_acquisition_time_ms = 5000; // Time to acquire fix (5 seconds)
    
public:
    myGPS(uart_inst_t *, int, int, int);
    void init();
    int readLine(std::string &);
    int readLine(std::string &, double &, char &, double &, char &, std::string &);
    int readLine(std::string &, double &, char &, double &, char &, std::string &, std::string &);
    std::string to_string(double, char, double, char, std::string &);
    
    // Fake GPS data methods
    void enableFakeGPS(bool enable) { use_fake_data = enable; }
    bool isFakeGPSEnabled() const { return use_fake_data; }
    void setFakeCoordinates(double lat, double lon) { 
        fake_latitude = lat; 
        fake_longitude = lon; 
    }
    
    // Get the current date from GPS in DDMMYY format
    // Returns empty string if no date is available
    std::string getDate() { return date; }
    
    // Tests GPS connection and returns a status code:
    // 0 = Good connection with valid NMEA data
    // 1 = Connected but no NMEA data received
    // 2 = UART communication error
    // 3 = Connected with NMEA data but with frame errors
    // 4 = Connected but baud rate likely incorrect
    int testConnection();
    
    // Returns the number of satellites currently visible to the GPS module
    int getVisibleSatellites();
    
    // Waits for a valid GPS fix with a specified timeout in seconds
    // Returns true if a fix was obtained, false if timeout occurred
    bool waitForFix(int timeout_seconds);
    
    // Sends hot start command to the GPS module
    // This resets the receiver but keeps ephemeris data for faster satellite acquisition
    // Returns true if command was sent successfully
    bool sendHotStartCommand();
    
    // Sends warm start command to the GPS module
    // This resets the receiver but keeps time, position, almanac, and ephemeris data
    // Returns true if command was sent successfully
    bool sendWarmStartCommand();
    
    // Sends a cold start command to the GPS module
    // This fully resets the receiver and clears all navigation data
    // May help when the receiver is completely stuck or has corrupt data
    // Returns true if command was sent successfully
    bool sendColdStartCommand();
    
    // Sends a command to the GPS module to specifically enable time messages
    // This can help ensure time data is being sent by the module
    bool enableTimeMessages();
    
    // Optimizes GPS module for faster fix acquisition by sending various configuration commands
    // Returns true if the GPS module is still responding after sending the commands
    bool optimizeForFastAcquisition();
};



#endif //MY_PROJECT_MYGPS_H
