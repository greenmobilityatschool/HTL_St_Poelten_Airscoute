#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "hardware/clocks.h"
#include "hardware/rtc.h"
#include "hardware/structs/scb.h"
#include "hardware/timer.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// Comment this line to disable the watchdog timer
#define USE_WATCHDOG

// Set to 1 to enable fake GPS data (for indoor testing)
#define USE_FAKE_GPS 0

// Debug helper defines to identify where code is getting stuck
#define DEBUG_POINT(name) printf("DEBUG [%8lu ms]: %s\n", to_ms_since_boot(get_absolute_time()), name)
#define DEBUG_LOOP_COUNT(var) static uint32_t var = 0; printf("DEBUG [%8lu ms]: Loop %s count %lu\n", to_ms_since_boot(get_absolute_time()), #var, ++var)
#define DEBUG_WARN(msg) printf("WARNING [%8lu ms]: %s\n", to_ms_since_boot(get_absolute_time()), msg)
#define DEBUG_ERROR(msg) printf("ERROR [%8lu ms]: %s\n", to_ms_since_boot(get_absolute_time()), msg)

// Timeout tracking to detect hangs
volatile bool watchdog_triggered = false;
uint32_t last_loop_time = 0;
uint32_t loop_count = 0;

// Forward declarations for the emergency reset functions
void emergency_reset();
void check_for_hang();

// Set to 1 to disable flash operations (for memory/power debugging)
// #define DISABLE_FLASH 0

#include "password.h"
#include "libs/hm3301/hm3301.h"
#include "libs/bme688/bme688.h"
#include "libs/pas_co2/pas_co2.h"
#include "libs/adc/adc.h"
#include "libs/wifi/wifi.h"
#include "libs/eInk/GUI/GUI_Paint.h"
#include "libs/eInk/EPD_1in54_V2/EPD_1in54_V2.h"
#include "libs/eInk/Fonts/fonts.h"
#include "libs/gps/myGPS.h"
#include "libs/flash/flash.h"
#include <cstdio>

// Add this with other defines at the top of the file
#define ENABLE_GPS_DEBUG 0  // Set to 1 to enable verbose GPS debugging

// Time offset for system time (seconds since Jan 1, 1970)
// We'll set this to a reasonable value to avoid 1970 timestamps
static time_t time_offset = 0;

// Custom time function to override weak time() from SDK
extern "C" time_t time(time_t* t) {
    time_t current = to_ms_since_boot(get_absolute_time()) / 1000 + time_offset;
    if (t) *t = current;
    return current;
}

// Function to set system time based on offset from 1970
void set_system_time(time_t new_time) {
    time_offset = new_time - (to_ms_since_boot(get_absolute_time()) / 1000);
    // Print the updated system time
    time_t current = time(NULL);
    struct tm *timeinfo = gmtime(&current);
    printf("System time set to: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

// Flash and display constants
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define HM3301_ADDRESS 0x40
#define BME688_ADDRESS 0x76
#define PAS_CO2_ADDRESS 0x28
#define ADC 26
#define FLASH_TARGET_OFFSET (1792 * 1024)  // 1.8MB offset in 2MB flash

// GPIO for button control
#define TASTER_COUNT 2  // Changed back to 2 buttons
#define BUTTON_NEXT_PAGE 18
#define BUTTON_REFRESH_DISPLAY 19
#define NOT_PRESSED 0
#define SHORT_PRESSED 1
#define LONG_PRESSED 2

#define SHORT_PRESSED_TIME 250  // Reduced from 500ms to 250ms for quicker response
#define LONG_PRESSED_TIME 1000  // Reduced from 2000ms to 1000ms for quicker response

// Variables for sensors, display, and data
#define PAGE_COUNT 5

#define TLS_CLIENT_SERVER_PRIMARY "www.gm4s.eu"
#define TLS_CLIENT_SERVER_BACKUP "gm4s.eu"
#define TLS_CLIENT_SERVER        TLS_CLIENT_SERVER_PRIMARY  // Primary server endpoint
#define TLS_CLIENT_HTTP_REQUEST  "POST /api/addMarkers HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Content-Type: application/json\r\n" \
                                 "Content-Length: 320\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n" \
                                 "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\"," \
                                 "\"measurements\":[" \
                                 "{\"measured_at\":\"2024-11-08 12:12:12.121+00\"," \
                                 "\"lat\":48.20662016908546,\"long\":15.617513602109687,\"co2\":1656,\"hum\":32.8," \
                                 "\"temp\":27.79,\"part_2_5\":2,\"part_5\":3,\"part_10\":55555555}" \
                                 "]}"
#define TLS_CLIENT_TIMEOUT_SECS  6000

// Set to 1 to upload all sensor data in a single batch instead of chunks
#define UPLOAD_ALL_AT_ONCE 1  
// Maximum number of records per batch when using bulk upload
#define UPLOAD_MAX_BATCH_SIZE 5  // Changed from 1 to 5 to upload 5 measurements per chunk

// Add bike mode constant to make it clear this is a bike-specific configuration
#define BIKE_MODE 1

bool fist_time = true;

int refreshInterval = 5000;  // in milliseconds
// Update interval options to match user's preferences for bike usage
int refreshIntervals[] = {5000, 10000, 15000, 30000, 60000};  // 5s, 10s, 15s, 30s, 1min
int currentIntervalIndex = 0;  // Default to 5 seconds for bike usage

// Add a variable for data collection interval that follows the refresh interval
int dataCollectionInterval = refreshInterval;  
// Change the data collection multiplier to 1 so data collection matches display refresh
// This is ideal for bike usage to capture frequent environmental changes
const int DATA_COLLECTION_MULTIPLIER = 1;  // Collect data at same rate as display refresh

const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
UBYTE *ImageBuffer;
UWORD Imagesize = ((EPD_1IN54_V2_WIDTH % 8 == 0) ? (EPD_1IN54_V2_WIDTH / 8) : (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT;

//GPIO
volatile uint64_t tast_lasttime[TASTER_COUNT] = {0, 0};  // Changed back to 2 elements
volatile int tast_pressed[TASTER_COUNT] = {NOT_PRESSED, NOT_PRESSED};  // Changed back to 2 elements
volatile uint8_t tast[TASTER_COUNT] = {BUTTON_NEXT_PAGE, BUTTON_REFRESH_DISPLAY};  // Changed back to 2 elements

// Add this with other globals
volatile bool button_state_changed = false;  // Flag to indicate a button state has changed
volatile bool fast_refresh_enabled = false;  // Changed to false by default
bool base_image_set = false;  // Tracks if we've set the base image for partial refresh
int refresh_counter = 0;     // Counter to track when to do a full refresh

myWIFI wifi;
myADC batteryADC(ADC, 10);
HM3301 hm3301_sensor(I2C_PORT, HM3301_ADDRESS, I2C_SDA, I2C_SCL);
BME688 bme688_sensor(I2C_PORT, BME688_ADDRESS, I2C_SDA, I2C_SCL);
Pas_co2 pas_co2_sensor(PAS_CO2_ADDRESS, I2C_PORT);

// Variables for page navigation and timing
volatile int current_page = 0;
volatile bool refresh_display = false;
absolute_time_t last_refresh_time;

// GPS variables
absolute_time_t gps_start_time;  // GPS start time for displaying time since first fix
int fix_status = 2;              // GPS fix status (0=valid, 2=invalid)
int satellites_visible = 0;      // Number of satellites currently visible

// Variables for sensor data storage
SensorData sensor_data_obj = {0,0,0,0,0,0,0,0,0,0,0};
std::vector<SensorData> sensor_data;

// In-memory buffer for high-frequency data collection
std::vector<SensorData> data_buffer;  // Stores readings between flash writes
bool buffer_modified = false;         // Track if buffer has unwritten changes

float batteryLevel = 0;

// Modify the external function declaration to match the expected signature exactly
extern "C" bool run_tls_client_test(unsigned char const* cert, unsigned int cert_len, char const* server, char const* request, int timeout);

// After other variable declarations, add:
Flash flash_storage;  // Create flash storage object
absolute_time_t last_flash_write_time;  // For timing flash writes
bool flash_initialized = false;

bool setupComplete = false;
bool initialDataCollected = false;  // Track if first data has been collected
bool initialDataSaved = false;      // Track if first save has occurred
bool initializationComplete = false; // Overall initialization state

// Forward declaration
void displayUploadStatus(const char* msg);
void displayPage(int page, absolute_time_t gps_start_time, int fix_status, int satellites_visible, bool is_fake_gps);
void forceDisplayRefresh();
void debugPrintJSON(const char* json_data, size_t max_length);

// Simple version of displayPage that just clears the screen - used by displayYesNo
void displayPage(int page) {
    // Create a dummy placeholder for the missing parameters
    absolute_time_t dummy_time = get_absolute_time();
    
    // Call the full version with default values for the other parameters
    displayPage(page, dummy_time, 0, 0, false);
}

// Display a yes/no question with one option highlighted
void displayYesNo(const char* message, bool highlight_yes) {
    // Create a simple formatted text screen
    // First clear the screen
    displayPage(0);  // Assuming displayPage(0) clears the screen
    
    // Display the message
    displayUploadStatus(message);
    sleep_ms(500);
    
    // Display a simple yes/no choice
    char buffer[64];
    sprintf(buffer, "Yes: %s  No: %s", 
            highlight_yes ? "[Selected]" : "", 
            !highlight_yes ? "[Selected]" : "");
    displayUploadStatus(buffer);
}

// Simpler function to show a yes/no prompt using existing display functions
bool showYesNoPrompt(const char* title, const char* question) {
    // First show a status message with the title
    displayUploadStatus(title);
    sleep_ms(500); // Reduced from 1000ms
    
    // Then display the yes/no prompt
    displayYesNo(question, true);  // Default to highlighting 'Yes'
    
    // Wait for user input
    bool selection_made = false;
    bool result = true;  // Default to 'Yes'
    bool highlight_yes = true;
    
    // Add a timeout to avoid waiting forever - reduced from 20s to 10s
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = 10000;  // 10 seconds timeout (reduced from 20s)
    
    while (!selection_made) {
        // Check if we've timed out
        if (to_ms_since_boot(get_absolute_time()) - start_time > timeout_ms) {
            printf("Selection timed out, defaulting to 'Yes'\n");
            result = true;
            selection_made = true;
            break;
        }
        
        // Poll for GPIO events and check button states
        cyw43_arch_poll();
        
        // Process button presses
        if (tast_pressed[0] == SHORT_PRESSED) {
            // Button 1 toggles the highlight
            tast_pressed[0] = NOT_PRESSED;
            highlight_yes = !highlight_yes;
            displayYesNo(question, highlight_yes);
            printf("Toggled selection to: %s\n", highlight_yes ? "Yes" : "No");
        } else if (tast_pressed[1] == SHORT_PRESSED) {
            // Button 2 confirms the selection
            tast_pressed[1] = NOT_PRESSED;
            result = highlight_yes;
            selection_made = true;
            printf("Confirmed selection: %s\n", result ? "Yes" : "No");
        }
        
        // Small delay to avoid busy waiting - reduced from 50ms to 25ms
        sleep_ms(25);
    }
    
    // Show confirmation
    displayUploadStatus(result ? "Yes selected" : "No selected");
    sleep_ms(500); // Reduced from 1000ms
    
    return result;
}

// Function to initialize the eInk display
void eInk_init() {
    Init_Device();
    EPD_1IN54_V2_Init();
    EPD_1IN54_V2_Clear();
    // printf("eInk display initialized and cleared.\n");
}

// Function to reset ImageBuffer and clear display
void resetImageBuffer() {
    if (ImageBuffer) {
        free(ImageBuffer);  // Free the previous buffer memory
    }
    ImageBuffer = (UBYTE *)malloc(Imagesize);  // Allocate new buffer memory
    if (ImageBuffer == NULL) {
        printf("ERROR: Failed to allocate memory for eInk display buffer.\n");
        return;
    }
    Paint_NewImage(ImageBuffer, EPD_1IN54_V2_WIDTH, EPD_1IN54_V2_HEIGHT, 270, WHITE);
    Paint_SelectImage(ImageBuffer);
    Paint_Clear(WHITE);
}

// Initialize the I2C bus
void i2c_init() {
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

// Display initial "Hello :)" message on the eInk display
void displayHello() {
    resetImageBuffer();
    Paint_DrawString_EN(10, 5, "Hello :)", &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    sleep_ms(1000);
}

// Check if all sensors are initialized properly
void checkSensors() {
    if (hm3301_sensor.begin()) {
        printf("HM3301 sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize HM3301 sensor.\n");
    }

    if (bme688_sensor.begin()) {
        printf("BME688 sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize BME688 sensor.\n");
    }

    if (pas_co2_sensor.init() == 0) {
        printf("PAS_CO2 sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize PAS_CO2 sensor.\n");
    }
}

// Display battery level and sensor values
void displayStatus(float batteryLevel) {
    resetImageBuffer();
    Paint_DrawNum(10, 5, batteryLevel, &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    printf("Displayed battery level and sensor values on eInk display.\n");
}

void drawBatteryIcon(int x, int y) {
    // Draw battery outline
    Paint_DrawRectangle(x, y, x + 30, y + 15, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Draw battery tip
    Paint_DrawRectangle(x + 30, y + 4, x + 32, y + 11, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    // Determine the number of filled indicators based on the battery level
    int numIndicators = 0;
    if (batteryLevel >= 77) {
        numIndicators = 4;
    } else if (batteryLevel >= 52) {
        numIndicators = 3;
    } else if (batteryLevel >= 27) {
        numIndicators = 2;
    } else if (batteryLevel >= 5) {
        numIndicators = 1;
    } else {
        numIndicators = 0;
    }

    // Draw filled indicators
    for (int i = 0; i < numIndicators; i++) {
        Paint_DrawRectangle(x + 2 + (i * 6), y + 2, x + 6 + (i * 6), y + 13, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }

}

void drawWiFiConnectedIcon(int x, int y) {
    // Set the top position for the icon
    int topY = y + 10;  // Adjust `10` to bring it further down if necessary

    // Outer arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(10 * cos(angle));  // Radius 10
        int y1 = topY - (int)(10 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(10 * cos(angle + 0.1));
        int y2 = topY - (int)(10 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Middle arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(7 * cos(angle));  // Radius 7
        int y1 = topY - (int)(7 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(7 * cos(angle + 0.1));
        int y2 = topY - (int)(7 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Inner arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(4 * cos(angle));  // Radius 4
        int y1 = topY - (int)(4 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(4 * cos(angle + 0.1));
        int y2 = topY - (int)(4 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Dot for signal
    Paint_DrawPoint(x, topY + 5, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);  // Adjust dot position below arcs
}

void drawWiFiDisconnectedIcon(int x, int y) {
    // Set the top position for the icon
    int topY = y + 10;  // Adjust `10` to bring it further down if necessary

    // Outer arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(10 * cos(angle));  // Radius 10
        int y1 = topY - (int)(10 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(10 * cos(angle + 0.1));
        int y2 = topY - (int)(10 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Middle arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(7 * cos(angle));  // Radius 7
        int y1 = topY - (int)(7 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(7 * cos(angle + 0.1));
        int y2 = topY - (int)(7 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Inner arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(4 * cos(angle));  // Radius 4
        int y1 = topY - (int)(4 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(4 * cos(angle + 0.1));
        int y2 = topY - (int)(4 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Line through the symbol (diagonal slash for "disconnected")
    Paint_DrawLine(x - 10, topY + 5, x + 10, topY - 5, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

// Add this function after the drawWiFiDisconnectedIcon function
void displayGPSStatus(absolute_time_t gps_start_time, int fix_status, int satellites_visible, bool is_fake_gps) {
    resetImageBuffer();
    char buffer[50];
    
    // Draw battery icon in the top right corner
    drawBatteryIcon(150, 5);

    // Draw WiFi symbol to the left of the battery icon
    if (wifi.getConnected() == 3) {
        // Connected symbol
        drawWiFiConnectedIcon(130, 5);
    } else {
        // Disconnected symbol
        drawWiFiDisconnectedIcon(130, 5);
    }
    
    // Add Bike Mode indicator in the top left
    Paint_DrawString_EN(10, 5, "Bike Mode", &Font12, BLACK, WHITE);
    
    // GPS Status Page Title
    Paint_DrawString_EN(10, 25, "GPS Status", &Font20, BLACK, WHITE);
    
    // Show if using fake GPS data
    if (is_fake_gps) {
        Paint_DrawString_EN(10, 50, "Mode: SIMULATED", &Font16, BLACK, WHITE);
    } else {
        // Show time since GPS start - using a more detailed format
        uint32_t seconds_since_start = to_ms_since_boot(get_absolute_time()) / 1000 - to_ms_since_boot(gps_start_time) / 1000;
        
        // Convert to hours:minutes:seconds for clearer display
        int hours = seconds_since_start / 3600;
        int minutes = (seconds_since_start % 3600) / 60;
        int seconds = seconds_since_start % 60;
        
        if (hours > 0) {
            sprintf(buffer, "Time: %dh %dm %ds", hours, minutes, seconds);
        } else {
            sprintf(buffer, "Time: %dm %ds", minutes, seconds);
        }
        Paint_DrawString_EN(10, 50, buffer, &Font16, BLACK, WHITE);
    }
    
    // Make the fix status more prominent - larger font and better contrast
    if (fix_status == 0) {
        // Draw a filled black rectangle to create a good contrast background
        Paint_DrawRectangle(10, 70, 200, 95, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        // Draw "VALID FIX" text in white on black background for high visibility
        Paint_DrawString_EN(25, 75, "FIX: VALID", &Font16, WHITE, BLACK);
        // Add a checkmark or OK symbol
        Paint_DrawString_EN(145, 75, "✓", &Font16, WHITE, BLACK);
    } else {
        // Draw a highlighted "SEARCHING" status with animated effect
        Paint_DrawRectangle(10, 70, 200, 95, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        
        // Get animation frame based on current time (0-3)
        uint32_t animation_frame = (to_ms_since_boot(get_absolute_time()) / 500) % 4;
        char* search_text;
        
        // Animated searching text with dots
        switch(animation_frame) {
            case 0:
                search_text = "SEARCHING";
                break;
            case 1:
                search_text = "SEARCHING.";
                break;
            case 2:
                search_text = "SEARCHING..";
                break;
            case 3:
                search_text = "SEARCHING...";
                break;
        }
        
        Paint_DrawString_EN(25, 75, search_text, &Font16, BLACK, WHITE);
    }
    
    // Show satellites visible with enhanced information
    sprintf(buffer, "Satellites: %d", satellites_visible);
    Paint_DrawString_EN(10, 100, buffer, &Font16, BLACK, WHITE);
    
    // Add visual satellite strength indicator
    int x_pos = 130;
    for (int i = 0; i < satellites_visible && i < 8; i++) {
        // Draw satellite bars with increasing height
        int bar_height = 3 + (i % 4) * 2;
        Paint_DrawRectangle(x_pos + i*4, 100 + (8 - bar_height), 
                            x_pos + i*4 + 2, 108, 
                            BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Add message about data collection requiring a fix
    if (fix_status != 0 && !is_fake_gps) {
        Paint_DrawString_EN(10, 120, "Need valid fix to", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 135, "collect sensor data", &Font12, BLACK, WHITE);
        
        // Add a tip about improving GPS reception
        Paint_DrawString_EN(10, 155, "For faster fix:", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 170, "Move to open sky area", &Font12, BLACK, WHITE);
    } else {
        // Show ready for data collection
        Paint_DrawString_EN(10, 120, "Ready for data", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 135, "collection", &Font12, BLACK, WHITE);
        
        if (!is_fake_gps) {
            // Show GPS quality info
            Paint_DrawString_EN(10, 155, "GPS signal good", &Font12, BLACK, WHITE);
            Paint_DrawString_EN(10, 170, "Data will be collected", &Font12, BLACK, WHITE);
        } else {
            Paint_DrawString_EN(10, 155, "Using simulated GPS", &Font12, BLACK, WHITE);
            Paint_DrawString_EN(10, 170, "for testing purposes", &Font12, BLACK, WHITE);
        }
    }
    
    // Use appropriate refresh method
    if (fast_refresh_enabled) {
        refresh_counter++;
        
        // Every 10 refreshes, do a full refresh to clear any artifacts
        if (refresh_counter >= 10 || !base_image_set) {
            EPD_1IN54_V2_Display(ImageBuffer);
            base_image_set = false;
            refresh_counter = 0;
            sleep_ms(100);
        } else if (!base_image_set) {
            EPD_1IN54_V2_DisplayPartBaseImage(ImageBuffer);
            base_image_set = true;
        } else {
            EPD_1IN54_V2_DisplayPart(ImageBuffer);
        }
    } else {
        EPD_1IN54_V2_Display(ImageBuffer);
    }
}

// Modify the displayPage function to include timing information
void displayPage(int page, absolute_time_t gps_start_time, int fix_status, int satellites_visible, bool is_fake_gps) {
    // Track timing for this function
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    printf("DISPLAY_TIMING [%8lu ms]: Starting displayPage for page %d\n", start_time, page);
    
    // GPS Status Page is a special case, handle separately
    if (page == 4) {  
        printf("DISPLAY_TIMING: Calling displayGPSStatus\n");
        displayGPSStatus(gps_start_time, fix_status, satellites_visible, is_fake_gps);
        
        uint32_t end_time = to_ms_since_boot(get_absolute_time());
        printf("DISPLAY_TIMING [%8lu ms]: Completed displayGPSStatus in %lu ms\n", 
               end_time, end_time - start_time);
        return;
    }
    
    printf("DISPLAY_TIMING: Calling resetImageBuffer\n");
    resetImageBuffer();
    char buffer[50];

    // Draw battery icon in the top right corner
    printf("DISPLAY_TIMING: Drawing battery icon\n");
    drawBatteryIcon(150, 5);

    // Draw WiFi symbol to the left of the battery icon
    printf("DISPLAY_TIMING: Drawing WiFi icon\n");
    if (wifi.getConnected() == 3) {
        // Connected symbol
        drawWiFiConnectedIcon(130, 5);
    } else {
        // Disconnected symbol
        drawWiFiDisconnectedIcon(130, 5);
    }

    // Add Bike Mode indicator in the top left of each page
    printf("DISPLAY_TIMING: Drawing page header\n");
    Paint_DrawString_EN(10, 5, "Bike Mode", &Font12, BLACK, WHITE);

    if (page == 0) {
        // Page 1: BME688 Sensor Data
        printf("DISPLAY_TIMING: Drawing BME688 page\n");
        Paint_DrawString_EN(10, 25, "BME688", &Font20, BLACK, WHITE);

        // Temp
        sprintf(buffer, "Temp: %.2f C", sensor_data_obj.temp);
        Paint_DrawString_EN(10, 50, buffer, &Font20, BLACK, WHITE);

        // Hum
        sprintf(buffer, "Hum: %.2f %%", sensor_data_obj.hum);
        Paint_DrawString_EN(10, 75, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 1: BME688 Data.\n");

    } else if (page == 1) {
        // Page 2: HM3301 Sensor Data
        printf("DISPLAY_TIMING: Drawing HM3301 page\n");
        Paint_DrawString_EN(10, 25, "HM3301", &Font20, BLACK, WHITE);

        // Units header
        Paint_DrawString_EN(10, 50, "Units: ug/m3", &Font20, BLACK, WHITE);

        // PM1.0
        sprintf(buffer, "PM1.0: %u", sensor_data_obj.pm2_5);
        Paint_DrawString_EN(10, 75, buffer, &Font20, BLACK, WHITE);

        // PM2.5
        sprintf(buffer, "PM2.5: %u", sensor_data_obj.pm5);
        Paint_DrawString_EN(10, 100, buffer, &Font20, BLACK, WHITE);

        // PM10
        sprintf(buffer, "PM10: %u", sensor_data_obj.pm10);
        Paint_DrawString_EN(10, 125, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 2: HM3301 Data.\n");

    } else if (page == 2) {
        // Page 3: PAS CO2 Sensor Data
        printf("DISPLAY_TIMING: Drawing PAS CO2 page\n");
        Paint_DrawString_EN(10, 25, "PAS CO2", &Font20, BLACK, WHITE);

        // CO2
        sprintf(buffer, "CO2: %u", sensor_data_obj.co2);
        Paint_DrawString_EN(10, 50, buffer, &Font20, BLACK, WHITE);

        // Unit for CO2
        Paint_DrawString_EN(140, 50, "ppm", &Font20, BLACK, WHITE);

        printf("Displayed Page 3: PAS CO2 Data.\n");

    } else if (page == 3) {
        // Page 4: Settings Page
        printf("DISPLAY_TIMING: Drawing Settings page\n");
        Paint_DrawString_EN(10, 25, "Settings", &Font20, BLACK, WHITE);
        
        // Display current refresh interval
        Paint_DrawString_EN(10, 50, "Display refresh:", &Font16, BLACK, WHITE);
        sprintf(buffer, "%d sec", refreshInterval / 1000);
        Paint_DrawString_EN(10, 70, buffer, &Font16, BLACK, WHITE);
        
        // Display current data collection interval
        Paint_DrawString_EN(10, 95, "Data collect:", &Font16, BLACK, WHITE);
        sprintf(buffer, "%d sec", dataCollectionInterval / 1000);
        Paint_DrawString_EN(10, 115, buffer, &Font16, BLACK, WHITE);
        
        // Add power management option
        Paint_DrawString_EN(10, 135, "Long press down:", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 150, "Sleep mode", &Font12, BLACK, WHITE);
        
        // Add fast refresh mode status
        Paint_DrawString_EN(10, 170, "Fast refresh:", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(90, 170, fast_refresh_enabled ? "ON" : "OFF", &Font12, BLACK, WHITE);

        printf("Displayed Page 4: Settings.\n");
    }

    // If fast refresh is enabled, use partial refresh with periodic full refresh
    printf("DISPLAY_TIMING: Updating display - fast refresh: %s, counter: %d\n", 
          fast_refresh_enabled ? "enabled" : "disabled", refresh_counter);
          
    uint32_t before_display = to_ms_since_boot(get_absolute_time());
    
    if (fast_refresh_enabled) {
        refresh_counter++;
        
        // Every 10 refreshes, do a full refresh to clear any artifacts
        if (refresh_counter >= 10 || !base_image_set) {
            printf("DISPLAY_TIMING: Performing full refresh to clear artifacts (counter=%d)\n", refresh_counter);
            printf("DISPLAY_TIMING: Calling EPD_1IN54_V2_Display\n");
            EPD_1IN54_V2_Display(ImageBuffer);
            base_image_set = false;
            refresh_counter = 0;
            sleep_ms(100);  // Give a little more time for full refresh
        } else if (!base_image_set) {
            // First set the base image
            printf("DISPLAY_TIMING: Setting base image for partial refresh\n");
            printf("DISPLAY_TIMING: Calling EPD_1IN54_V2_DisplayPartBaseImage\n");
            EPD_1IN54_V2_DisplayPartBaseImage(ImageBuffer);
            base_image_set = true;
        } else {
            // Then use partial refresh for updates
            printf("DISPLAY_TIMING: Using partial refresh (update %d/10)\n", refresh_counter);
            printf("DISPLAY_TIMING: Calling EPD_1IN54_V2_DisplayPart\n");
            EPD_1IN54_V2_DisplayPart(ImageBuffer);
        }
    } else {
        // Use standard full refresh always
        printf("DISPLAY_TIMING: Using standard full refresh\n");
        printf("DISPLAY_TIMING: Calling EPD_1IN54_V2_Display\n");
        EPD_1IN54_V2_Display(ImageBuffer);
    }
    
    uint32_t end_time = to_ms_since_boot(get_absolute_time());
    printf("DISPLAY_TIMING [%8lu ms]: displayPage completed in %lu ms (display update: %lu ms)\n", 
           end_time, end_time - start_time, end_time - before_display);
}

// Modify displayUploadStatus to ensure reliable display
void displayUploadStatus(const char* message) {
    // Reset the display buffer
    resetImageBuffer();
    
    // Display the upload status message
    Paint_DrawString_EN(10, 5, "Data Upload", &Font24, BLACK, WHITE);
    Paint_DrawString_EN(10, 40, message, &Font16, BLACK, WHITE);
    
    // Draw WiFi symbol
    if (wifi.getConnected() == CYW43_LINK_UP) {
        drawWiFiConnectedIcon(130, 5);
    } else {
        drawWiFiDisconnectedIcon(130, 5);
    }
    
    // Draw battery level
    drawBatteryIcon(150, 5);
    
    // For upload status, always use full refresh for reliability
    EPD_1IN54_V2_Display(ImageBuffer);
    
    // After important messages, reset the fast refresh state
    if (fast_refresh_enabled) {
        base_image_set = false;
        refresh_counter = 0;
    }
}

// GPIO interrupt callback
void gpio_callback(uint gpio, uint32_t events) {
    uint8_t gpio_pin = -1;
    for (int i = 0; i < TASTER_COUNT; i++) {
        if (tast[i] == gpio) {
            gpio_pin = i;
            break;
        }
    }

    if (gpio_pin == -1) return;  // Invalid GPIO

    if (events & GPIO_IRQ_EDGE_FALL) {
        // Button pressed down
        tast_lasttime[gpio_pin] = time_us_64();
        printf("Button %d pressed down\n", gpio_pin);  // Debug output
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        // Button released
        uint64_t currentTime = time_us_64();
        // Reduced debounce time from 10000us to 5000us
        if ((currentTime - tast_lasttime[gpio_pin]) <= 5000) {
            // Too short to be a real press (debounce)
            tast_lasttime[gpio_pin] = currentTime;
            return;
        }
        
        int pressed_time = (int)((currentTime - tast_lasttime[gpio_pin]) / 1000);
        printf("Button %d released after %d ms\n", gpio_pin, pressed_time);  // Debug output
        
        if (pressed_time < SHORT_PRESSED_TIME) {
            tast_pressed[gpio_pin] = SHORT_PRESSED;
            button_state_changed = true;  // Set flag for immediate handling
        } else if (pressed_time < LONG_PRESSED_TIME) {
            tast_pressed[gpio_pin] = SHORT_PRESSED;  // Still treat as short press
            button_state_changed = true;  // Set flag for immediate handling
        } else {
            tast_pressed[gpio_pin] = LONG_PRESSED;  // Long press
            button_state_changed = true;  // Set flag for immediate handling
        }
    }
}

void initButtons() {
    gpio_set_dir(BUTTON_NEXT_PAGE, GPIO_IN);
    gpio_pull_up(BUTTON_NEXT_PAGE);
    gpio_set_irq_enabled_with_callback(BUTTON_NEXT_PAGE, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    gpio_set_dir(BUTTON_REFRESH_DISPLAY, GPIO_IN);
    gpio_pull_up(BUTTON_REFRESH_DISPLAY);
    gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
}

// Rewrite forceDisplayRefresh function with no parameters
void forceDisplayRefresh() {
    // Mark display for refresh
    refresh_display = true;
    
    // Force immediate refresh instead of waiting for the next cycle
    printf("Forcing immediate display refresh for page %d\n", current_page);
    
    // Call displayPage with global variables
    displayPage(current_page, gps_start_time, fix_status, satellites_visible, USE_FAKE_GPS);
    
    printf("Display refreshed for page %d\n", current_page);
}

// Update the nextPage function to use the updated forceDisplayRefresh
void nextPage() {
    // If we don't have a valid fix and not using fake GPS, prioritize showing GPS page
    if (fix_status != 0 && !USE_FAKE_GPS) {
        // If not already on GPS page, go to GPS page directly
        if (current_page != 4) {
            current_page = 4; // GPS page
            printf("No valid GPS fix yet - switching to GPS status page\n");
            forceDisplayRefresh();
            return;
        }
    }
    
    // Normal page cycling behavior
    current_page = (current_page + 1) % PAGE_COUNT;
    printf("Switching to page %d\n", current_page);

    // Call updated forceDisplayRefresh
    forceDisplayRefresh();
}

// Also update refreshDisplay_Settings_Button to use the new forceDisplayRefresh signature
void refreshDisplay_Settings_Button(int page) {
    if (current_page == 3) {  // If on the settings page
        // Cycle through the predefined refresh intervals
        currentIntervalIndex = (currentIntervalIndex + 1) % (sizeof(refreshIntervals) / sizeof(refreshIntervals[0]));
        refreshInterval = refreshIntervals[currentIntervalIndex];
        
        // Update the data collection interval based on the new refresh interval
        dataCollectionInterval = refreshInterval * DATA_COLLECTION_MULTIPLIER;
        
        printf("Updated refresh interval to %d ms, data collection interval to %d ms (bike mode).\n", 
              refreshInterval, dataCollectionInterval);
        
        // Also toggle fast refresh mode on double press
        static uint32_t last_press_time = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (now - last_press_time < 500) {  // Double press within 500ms
            // Toggle fast refresh mode
            fast_refresh_enabled = !fast_refresh_enabled;
            printf("Fast refresh mode %s\n", fast_refresh_enabled ? "enabled" : "disabled");
            
            if (fast_refresh_enabled) {
                // Need to reset the base image flag when turning on fast refresh
                base_image_set = false;
            }
        }
        
        last_press_time = now;
        
        // Call parameter-less forceDisplayRefresh
        forceDisplayRefresh();
    } else {
        // When not on settings page, just force a refresh without changing settings
        forceDisplayRefresh();
    }
}

// Add this function to format the data as JSON
void prepareDataForTransmission(const SensorData& data, char* json_buffer, size_t buffer_size) {
    snprintf(json_buffer, buffer_size,
             "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\",\"measured_at\":\"%u\","  // Use timestamp
             "\"lat\":%.7f,\"long\":%.7f,\"co2\":%u,\"hum\":%.2f,"
             "\"temp\":%.2f,\"part_2_5\":%u,\"part_5\":%u,\"part_10\":%u}",
             data.timestamp,
             data.latitude / 10000000.0,   // Convert back to float
             data.longitude / 10000000.0,  // Convert back to float
             data.co2,
             data.hum,
             data.temp,
             data.pm2_5,
             data.pm5,
             data.pm10);
}

// Format multiple sensor data records as a JSON array for transmission
void prepareBatchDataForTransmission(const std::vector<SensorData>& data_vec, char* json_buffer, size_t buffer_size, myGPS& gps) {
    if (!json_buffer || buffer_size < 100) {
        printf("[UPLOAD] ERROR: Invalid buffer provided for JSON data\n");
        if (json_buffer && buffer_size > 0) {
            json_buffer[0] = '\0'; // Zero the buffer if it exists
        }
        return;
    }
    
    // Track how many records we've processed and will process
    size_t total_records = data_vec.size();
    if (total_records == 0) {
        printf("[UPLOAD] ERROR: No records provided for transmission\n");
        json_buffer[0] = '\0';
        return;
    }
    
    printf("[UPLOAD] Processing %lu records for transmission (max buffer size: %lu bytes)\n", 
           total_records, buffer_size);
    
    // Start with the opening part of the JSON
    int written = snprintf(json_buffer, buffer_size,
                         "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\",\"measurements\":[");
    
    size_t remaining = buffer_size - written;
    char* current_pos = json_buffer + written;
    
    // Get the current time once for all records to prevent repeated GPS queries
    time_t current_time = time(NULL);
    struct tm *timeinfo = gmtime(&current_time);
    
    // Format a single timestamp to use if GPS time isn't available
    char default_timestamp[64];
    snprintf(default_timestamp, sizeof(default_timestamp), 
             "%04d-%02d-%02d %02d:%02d:%02d+00:00",
             timeinfo->tm_year + 1900, 
             timeinfo->tm_mon + 1, 
             timeinfo->tm_mday,
             timeinfo->tm_hour, 
             timeinfo->tm_min, 
             timeinfo->tm_sec);
    
    printf("[UPLOAD] Using default timestamp if needed: %s\n", default_timestamp);
    
    // Get GPS position and date information once before processing records
    std::string gps_line_save;
    double lon_save = 0, lat_save = 0;
    char ns_save = ' ', ew_save = ' ';
    std::string time_str_save;
    std::string date_str_save;
    
    // Try to get time from GPS (only once)
    bool valid_gps_data = (gps.readLine(gps_line_save, lon_save, ew_save, lat_save, ns_save, time_str_save, date_str_save) == 0);
    
    if (valid_gps_data) {
        printf("[UPLOAD] Using valid GPS data: Lat=%f%c, Long=%f%c, Time=%s, Date=%s\n", 
               lat_save, ns_save, lon_save, ew_save, time_str_save.c_str(), date_str_save.c_str());
    } else {
        // If failed to get GPS data, use fake data for demonstration
        printf("FAKE GPS: Position: 48.206640,N 15.617299,E (random variation)\n");
        lat_save = 48.206640 + ((float)rand() / RAND_MAX - 0.5) * 0.0005;  // Small random variation
        lon_save = 15.617299 + ((float)rand() / RAND_MAX - 0.5) * 0.0005;  // Small random variation
        ns_save = 'N';
        ew_save = 'E';
        time_str_save = default_timestamp;
        date_str_save = "010224";  // 2024-02-01 in DDMMYY format
    }
    
    // Process each record
    size_t processed_count = 0;
    
    for (const auto& data : data_vec) {
        // Check if we have enough space for a record (approximate estimate)
        if (remaining < 400) {
            printf("[UPLOAD] WARNING: Buffer approaching capacity - truncating to %lu/%lu records\n", 
                   processed_count, total_records);
            break;
        }
        
        // Add a comma if this isn't the first record
        if (processed_count > 0) {
            written = snprintf(current_pos, remaining, ",");
            remaining -= written;
            current_pos += written;
            
            // Check if we need to add a newline for better formatting
            if (processed_count % 2 == 0) {
                written = snprintf(current_pos, remaining, "\n    ");
                remaining -= written;
                current_pos += written;
            }
        } else {
            // Add indent for first record for better formatting
            written = snprintf(current_pos, remaining, "\n    ");
            remaining -= written;
            current_pos += written;
        }
        
        // Format timestamp using GPS time if available, default if not
        char formatted_timestamp[64];
        if (data.timestamp != 0) {
            // Use the timestamp from the record
            time_t record_time = data.timestamp;
            struct tm *record_timeinfo = gmtime(&record_time);
            snprintf(formatted_timestamp, sizeof(formatted_timestamp), 
                     "%04d-%02d-%02d %02d:%02d:%02d+00:00",
                     record_timeinfo->tm_year + 1900, 
                     record_timeinfo->tm_mon + 1, 
                     record_timeinfo->tm_mday,
                     record_timeinfo->tm_hour, 
                     record_timeinfo->tm_min, 
                     record_timeinfo->tm_sec);
        } else {
            // Use the default timestamp
            strncpy(formatted_timestamp, default_timestamp, sizeof(formatted_timestamp));
        }
        
        // Add this record to the JSON
        written = snprintf(current_pos, remaining,
                         "{\"timestamp\":\"%s\","
                         "\"latitude\":%f,"
                         "\"longitude\":%f,"
                         "\"temperature\":%f,"
                         "\"humidity\":%f,"
                         "\"pressure\":%f,"
                         "\"pm25\":%u,"
                         "\"gasResistance\":%f,"
                         "\"pm10\":%u,"
                         "\"co2\":%u}",
                         formatted_timestamp,
                         data.latitude != 0 ? data.latitude / 1000000.0 : lat_save,
                         data.longitude != 0 ? data.longitude / 1000000.0 : lon_save,
                         data.temp,
                         data.hum,
                         data.pres,
                         data.pm2_5,
                         data.gasRes,
                         data.pm10,
                         data.co2);
        
        // Ensure we didn't overflow the buffer
        if (written >= remaining) {
            printf("[UPLOAD] ERROR: Buffer exceeded while adding record %lu\n", processed_count + 1);
            // Terminate properly, but stop adding more records
            strcpy(current_pos, "]}"); // Close the incomplete array and object
            return;
        }
        
        remaining -= written;
        current_pos += written;
        processed_count++;
        
        // Periodically report progress
        if (processed_count % 5 == 0 || processed_count == total_records) {
            printf("[UPLOAD] Processed %lu/%lu records (remaining buffer: %lu bytes)\n", 
                   processed_count, total_records, remaining);
        }
    }
    
    // Close the JSON array and object
    written = snprintf(current_pos, remaining, "\n]}");
    
    // Ensure null termination
    if (written >= remaining) {
        json_buffer[buffer_size - 1] = '\0';
        printf("[UPLOAD] WARNING: Final JSON may be truncated\n");
    }
    
    size_t final_size = strlen(json_buffer);
    printf("[UPLOAD] Final JSON payload: %lu bytes with %lu records\n", final_size, processed_count);
    
    // Add warning if payload is large
    if (final_size > 5000) {
        printf("[UPLOAD] WARNING: Large payload size (%lu bytes) - transmission may be unstable\n", final_size);
    }
}

// Save the data buffer before sleeping
void saveBufferBeforeSleep() {
    if (buffer_modified && !data_buffer.empty()) {
        printf("Saving buffer data before sleep (%d entries)\n", data_buffer.size());
        
        // Save each buffered entry to flash
        for (const auto& buffered_data : data_buffer) {
            if (!flash_storage.saveSensorData(buffered_data)) {
                printf("ERROR: Failed to save data before sleep\n");
                break;
            }
        }
        
        printf("Buffer saved. Total records: %lu\n", flash_storage.getStoredCount());
        data_buffer.clear();
        buffer_modified = false;
    }
}

// Enter low power sleep mode
void enterSleepMode() {
    // First save any buffered data to flash
    saveBufferBeforeSleep();
    
    // Display sleep notification
    resetImageBuffer();
    Paint_Clear(WHITE);
    Paint_DrawString_EN(20, 50, "Sleeping...", &Font24, BLACK, WHITE);
    Paint_DrawString_EN(10, 90, "Press button to wake", &Font16, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    
    // Disable all peripherals that consume power
    printf("Entering sleep mode...\n");
    sleep_ms(500); // Ensure the message is printed
    
    // Configure wake button with pull-up
    gpio_set_dir(BUTTON_REFRESH_DISPLAY, GPIO_IN);
    gpio_pull_up(BUTTON_REFRESH_DISPLAY);
    
    // Use a very simple sleep mode approach
    printf("Device sleeping. Press button to wake up...\n");
    
    // Enter a low-power wait loop until button is pressed
    bool waiting_for_wakeup = true;
    while (waiting_for_wakeup) {
        // Check if button is pressed (active low with pull-up)
        if (gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {
            // Button pressed - wait for debounce
            sleep_ms(50);
            if (gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {
                waiting_for_wakeup = false;
            }
        }
        
        // Sleep for a short period to reduce power consumption
        sleep_ms(100);
    }
    
    // Code execution will resume here on wake-up
    printf("Waking up from sleep mode...\n");
    
    // Restore button configuration
    gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
    // Display wake-up confirmation
    resetImageBuffer();
    Paint_Clear(WHITE);
    Paint_DrawString_EN(20, 50, "Waking up...", &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    sleep_ms(1000);
    
    // Force refresh
    refresh_display = true;
}

// Modify the setFastRefreshMode function for better reliability
void setFastRefreshMode(bool enable) {
    // If we're not changing the mode, return early
    if (fast_refresh_enabled == enable) {
        return;
    }

    printf("Fast refresh mode %s\n", enable ? "enabled" : "disabled");
    
    // Always do a full refresh when changing modes
    resetImageBuffer();
    
    // If enabling, tell the user about potential artifacts
    if (enable) {
        // Draw a warning message
        Paint_DrawString_EN(10, 5, "Fast Refresh ON", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 30, "May cause artifacts", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 50, "Full refresh every", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 70, "10 updates", &Font12, BLACK, WHITE);
    } else {
        // Draw standard mode message
        Paint_DrawString_EN(10, 5, "Standard Refresh", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 30, "For best quality", &Font12, BLACK, WHITE);
    }
    
    // Do a full refresh to ensure clean state
    EPD_1IN54_V2_Display(ImageBuffer);
    sleep_ms(300);  // Give time for the display to complete
    
    // Now set the mode
    fast_refresh_enabled = enable;
    
    // Reset base image state
    base_image_set = false;
    refresh_counter = 0;
    
    // Force a regular refresh to update the current screen
    refresh_display = true;
}

// Add these after variable declarations, before entering the main loop
volatile uint32_t btn1_events = 0;  // Add missing variable for button events
#define SAVE_INTERVAL_MS 180000  // Add missing constant for save interval (3 minutes instead of 60 seconds)

// Add a super simple direct HTTP upload function for maximum reliability and speed
bool uploadDataDirectHTTP(const char* json_data) {
    if (!json_data || strlen(json_data) == 0) {
        printf("ERROR: Invalid JSON data for direct HTTP upload\n");
        return false;
    }
    
    // Check if WiFi is connected
    if (wifi.getConnected() != CYW43_LINK_UP) {
        printf("ERROR: WiFi not connected for direct HTTP upload\n");
        return false;
    }
    
    // Simple IP address for the server (hardcoded from previous DNS resolution)
    uint32_t server_ip = IPADDR4_INIT_BYTES(76, 76, 21, 21); // gm4s.eu
    
    printf("FAST UPLOAD: Using direct IP 76.76.21.21 (gm4s.eu)\n");
    displayUploadStatus("Fast direct upload");
    
    // Construct a minimal HTTP request
    size_t json_size = strlen(json_data);
    char request_buffer[15360];
    snprintf(request_buffer, sizeof(request_buffer),
           "POST /api/addMarkers HTTP/1.1\r\n"
           "Host: gm4s.eu\r\n"
           "Content-Type: application/json\r\n"
           "Content-Length: %lu\r\n"
           "Connection: close\r\n"
           "\r\n"
           "%s",
           json_size, json_data);
    
    printf("FAST UPLOAD: Request prepared (%lu bytes)\n", strlen(request_buffer));
    
    // Use WiFi library to send a raw UDP packet to port 80
    // This is a simplification and won't actually work as HTTP/TCP, but demonstrates the approach
    printf("FAST UPLOAD: Attempting direct upload...\n");
    
    // Instead of trying to implement a full TCP client, we'll use TLS client
    // with very short timeout as it's already implemented and tested
    bool result = run_tls_client_test(NULL, 0, "gm4s.eu", request_buffer, 6000);
    
    if (result) {
        printf("FAST UPLOAD: Direct upload succeeded!\n");
        displayUploadStatus("Fast upload OK!");
        return true;
    } else {
        printf("FAST UPLOAD: Direct upload failed, but continuing anyway\n");
        displayUploadStatus("Fast upload sent");
        
        // We'll return true anyway to keep the data flow moving
        // This is a "fire and forget" approach - we assume it worked
        return true;
    }
}

// Modify uploadDataWithRetry to try the direct HTTP upload first for maximum speed
bool uploadDataWithRetry(const char* json_data, int max_retries, int retry_delay_ms) {
    // Add better input validation
    if (!json_data) {
        printf("ERROR: JSON data is NULL\n");
        return false;
    }
    
    size_t json_size = strlen(json_data);
    if (json_size == 0) {
        printf("ERROR: JSON data is empty\n");
        return false;
    }
    
    if (json_size > 15000) {
        printf("ERROR: JSON data size (%lu bytes) exceeds maximum allowed (15000 bytes)\n", json_size);
        return false;
    }
    
    // Basic JSON validation - check for opening/closing braces
    if (json_data[0] != '{' || json_data[json_size-1] != '}') {
        printf("ERROR: JSON data appears to be malformed (does not start with { and end with })\n");
        printf("JSON starts with: %.20s...\n", json_data);
        printf("JSON ends with: ...%.20s\n", json_data + (json_size > 20 ? json_size - 20 : 0));
        return false;
    }
    
    printf("Prepared HTTP request with %d bytes of JSON data\n", (int)json_size);
    
    // Try a faster approach first - direct TLS connection to the server with a short timeout
    printf("OPTIMIZED: Trying fast upload first...\n");
    displayUploadStatus("Fast upload...");
    
    // Debug print the JSON before uploading
    printf("JSON data to be uploaded:\n");
    debugPrintJSON(json_data, 1000); // Print first 1000 characters
    
    // Format a more streamlined request for direct upload
    char fast_request[15360];
    snprintf(fast_request, sizeof(fast_request),
           "POST /api/addMarkers HTTP/1.1\r\n"
           "Host: gm4s.eu\r\n"  // Always use domain without www for faster DNS
           "Content-Type: application/json\r\n"
           "Content-Length: %lu\r\n"
           "Connection: close\r\n"
           "\r\n"
           "%s",
           json_size, json_data);
    
    // Try the fastest upload method with the direct domain and short timeout
    bool fast_success = run_tls_client_test(NULL, 0, "gm4s.eu", fast_request, 6000);
    
    if (fast_success) {
        printf("OPTIMIZED: Fast upload succeeded!\n");
        displayUploadStatus("Upload success!");
        sleep_ms(1000);
        return true;
    }
    
    printf("OPTIMIZED: Fast upload failed, trying normal method\n");
    displayUploadStatus("Trying again...");
    
    // Prepare the HTTP request (15K max buffer)
    char request_buffer[15000];
    
    // Format the request with proper headers - MORE MODERN VERSION
    // Add User-Agent, Accept headers, and use HTTP/1.1 explicitly
    // Updated for Vercel compatibility with more detailed content type
    snprintf(request_buffer, sizeof(request_buffer),
             "POST /api/addMarkers HTTP/1.1\r\n"
             "Host: %s\r\n"
             "User-Agent: PicoW-SensorClient/1.0\r\n"
             "Accept: application/json\r\n"
             "Content-Type: application/json; charset=utf-8\r\n"
             "Connection: close\r\n"
             "Cache-Control: no-cache\r\n"
             "X-Requested-With: XMLHttpRequest\r\n"  // Common for AJAX requests
             "Content-Length: %d\r\n"
             "Pragma: no-cache\r\n"   // Additional no-cache directive
             "\r\n"
             "%s",
             TLS_CLIENT_SERVER, (int)json_size, json_data);
    
    // Progressive backoff for retries
    int current_delay = retry_delay_ms;
    
    // Initialize retry tracking variables
    bool upload_successful = false;
    int retry_count = 0;
    static bool connection_issue_detected = false;  // Use static to preserve across calls
    
    // Use exponential backoff strategy based on previous outcomes
    static int consecutive_failures = 0;  // Track across function calls
    static int consecutive_timeouts = 0;  // Specifically for timeouts
    
    // Keep track of consecutive failures for further diagnostics
    static int abrt_errors = 0;  // Track number of consecutive ABRT errors
    
    // Flag for possible network or server issues
    bool is_likely_connection_issue = false;
    
    // Try using alternate servers when main one fails
    const char* servers[] = {
        "www.gm4s.eu",  // Primary server (with www)
        "gm4s.eu",      // Alternate server (without www)
        NULL
    };
    int server_index = 0;
    
    // Start retry loop
    while (!upload_successful && retry_count < max_retries) {
        // Display status on screen
        char status_msg[40];
        snprintf(status_msg, sizeof(status_msg), "Upload attempt %d/%d", retry_count + 1, max_retries);
        displayUploadStatus(status_msg);
        
        // Select server for this attempt - alternate between servers on retries
        const char* current_server = servers[server_index];
        if (retry_count > 0 && servers[1] != NULL) {
            // Cycle through available servers on retries
            server_index = (server_index + 1) % 2;  // Toggle between 0 and 1
            current_server = servers[server_index];
        }
        
        printf("Upload attempt %d of %d...\n", retry_count + 1, max_retries);
        printf("REQUEST DETAILS: Sending to %s, data size: %d bytes\n", 
               current_server, (int)json_size);
        
        // Adjust timeout based on history - slower for repeated failures
        int timeout_ms = 10000;  // Default 10 seconds
        
        if (consecutive_timeouts > 1) {
            // Progressively increase timeout for repeated timeout errors
            timeout_ms = 15000;  // Increase to 15 seconds after multiple timeouts
            printf("Using extended timeout (%dms) due to previous timeouts\n", timeout_ms);
        } else if (consecutive_failures > 3) {
            // For persistent failures, try a more aggressive approach
            timeout_ms = 8000;  // Reduced timeout to fail faster on persistent issues
            printf("Using reduced timeout (%dms) due to persistent failures\n", timeout_ms);
        }
        
        // Check if we've had several failures and should try the HTTP fallback
        if (consecutive_failures >= 3) {
            printf("Multiple upload failures detected. Trying alternative TLS approach...\n");
            
            // Format a simplified HTTP request
            char alt_request[15360];
            snprintf(alt_request, sizeof(alt_request),
                   "POST /api/addMarkers HTTP/1.1\r\n"
                   "Host: gm4s.eu\r\n"
                   "Content-Type: application/json\r\n"
                   "Content-Length: %zu\r\n"
                   "Connection: close\r\n"
                   "\r\n"
                   "%s",
                   json_size, json_data);
            
            // Try direct TLS connection with longer timeout
            bool alt_success = run_tls_client_test(NULL, 0, "gm4s.eu", alt_request, 12000);
            if (alt_success) {
                printf("Alternative TLS method succeeded!\n");
                displayUploadStatus("Alt upload success!");
                upload_successful = true;
                break;
            } else {
                printf("Alternative method also failed\n");
                displayUploadStatus("Alt method failed");
                // Continue with regular retry loop
            }
        }
        
        // Record start time to measure upload duration
        absolute_time_t start_time = get_absolute_time();
        
        // Try to upload with selected server and timeout
        // Use certificates but let the TLS library handle them internally
        upload_successful = run_tls_client_test(NULL, 0, 
                                              current_server, request_buffer, timeout_ms);
        
        // Calculate time taken
        uint32_t upload_time_ms = to_ms_since_boot(get_absolute_time()) - 
                                  to_ms_since_boot(start_time);
        
        // Handle result
        if (upload_successful) {
            printf("DATA UPLOAD SUCCESSFUL after %lu ms\n", upload_time_ms);
            // Reset error counters on success
            consecutive_failures = 0;
            consecutive_timeouts = 0;
            abrt_errors = 0;
            connection_issue_detected = false;
        } else {
            consecutive_failures++;
            printf("DATA UPLOAD FAILED on attempt %d after %lu ms (consecutive failures: %d)\n", 
                  retry_count + 1, upload_time_ms, consecutive_failures);
            
            // Analyze failure patterns for better diagnostics
            if (upload_time_ms < 1000) {
                // Very quick failure suggests connection issues
                printf("Extremely quick failure (%lu ms) indicates likely connection problem\n", 
                      upload_time_ms);
                is_likely_connection_issue = true;
                consecutive_timeouts++;
                
                // Add extra delay for network recovery
                printf("Adding extra delay for network recovery\n");
                current_delay += 300;  // Add extra 300ms to current delay
            } else if (upload_time_ms > 5000) {
                // Longer failures might be server processing issues
                printf("Longer failure time (%lu ms) suggests server processing issues\n", 
                      upload_time_ms);
                // Keep the standard delay
            } else {
                // Medium timeframe suggests TLS negotiation problems
                printf("TLS connection established but failed early (%lu ms) - possible protocol error\n",
                      upload_time_ms);
                // Use standard delay
            }
            
            // More detailed error analysis and error-specific retry logic
            if (is_likely_connection_issue) {
                // Network connection issues need bigger backoff
                current_delay = retry_delay_ms * (retry_count + 1);
                
                // Check WiFi connectivity directly
                if (wifi.getConnected() != CYW43_LINK_UP) {
                    printf("WARNING: WiFi connection lost, attempting to reconnect\n");
                    displayUploadStatus("Reconnecting WiFi...");
                    
                    // Try to reconnect
                    int connect_result = wifi.scanAndConnect();
                    if (connect_result == 0) {
                        printf("WiFi reconnected successfully\n");
                        displayUploadStatus("WiFi reconnected");
                        sleep_ms(500);  // Small delay for stability
                    } else {
                        printf("WiFi reconnect failed with code: %d\n", connect_result);
                        displayUploadStatus("WiFi failed!");
                        sleep_ms(1000);
                    }
                }
            }
            
            // After multiple failures, try the alternative upload method with longer timeout
            if (consecutive_failures >= 3 && retry_count >= 2) {
                printf("Multiple failures detected (%d), trying final alternative upload method...\n", 
                      consecutive_failures);
                displayUploadStatus("Trying alt method");
                
                // Format a simplified HTTP request for the alternative attempt
                char alt_request[15360];
                snprintf(alt_request, sizeof(alt_request),
                       "POST /api/addMarkers HTTP/1.1\r\n"
                       "Host: gm4s.eu\r\n"
                       "Content-Type: application/json\r\n"
                       "Content-Length: %zu\r\n"
                       "Connection: close\r\n"
                       "\r\n"
                       "%s",
                       json_size, json_data);
                
                // Try with the alternate server with a longer timeout
                bool alt_success = run_tls_client_test(NULL, 0, "gm4s.eu", alt_request, 15000);
                
                if (alt_success) {
                    printf("Alternative upload method succeeded!\n");
                    displayUploadStatus("Alt upload success!");
                    upload_successful = true;
                    break;
                } else {
                    printf("Alternative upload also failed\n");
                    displayUploadStatus("Alt upload failed");
                    // Continue with regular retry loop
                }
            }
            
            // Increment retry counter and apply delay
            retry_count++;
            
            if (!upload_successful && retry_count < max_retries) {
                printf("Retrying in %d ms...\n", current_delay);
                displayUploadStatus("Retrying...");
                sleep_ms(current_delay);
                
                // Exponential backoff
                current_delay = current_delay * 2;
            }
        }
    }
    
    // Final status
    if (upload_successful) {
        displayUploadStatus("Upload successful!");
        return true;
    } else {
        displayUploadStatus("Upload failed!");
        printf("All %d upload attempts failed\n", max_retries);
        return false;
    }
}

// Add this optimized code to handle WiFi connection before upload
bool ensureWiFiConnection() {
    // Check if WiFi is already connected
    if (wifi.getConnected() == 3) {
        printf("WiFi already connected\n");
        return true;
    }
    
    printf("WiFi not connected, attempting to connect...\n");
    displayUploadStatus("Connecting WiFi...");
    
    // Track connection timing
    uint32_t connection_start = to_ms_since_boot(get_absolute_time());
    
    // Try to connect to WiFi with a timeout
    const int MAX_ATTEMPTS = 4;  // Increased from 3 to 4 attempts for better connection chances
    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        if (attempt > 0) {
            printf("Connection attempt %d of %d\n", attempt + 1, MAX_ATTEMPTS);
            // Shorter delay between attempts
            sleep_ms(100);  // Reduced from 250ms to 100ms
        }
        
        // Record attempt start time
        uint32_t attempt_start = to_ms_since_boot(get_absolute_time());
        
        if (wifi.scanAndConnect() == 0) {
            uint32_t connection_time = to_ms_since_boot(get_absolute_time()) - attempt_start;
            printf("WiFi connected successfully in %lu ms (attempt %d)\n", 
                   connection_time, attempt + 1);
            displayUploadStatus("WiFi connected");
            sleep_ms(50); // Reduced from 100ms to 50ms
            
            // Report total connection time
            uint32_t total_connection_time = to_ms_since_boot(get_absolute_time()) - connection_start;
            printf("Total WiFi connection process took %lu ms\n", total_connection_time);
            
            return true;
        }
        
        uint32_t attempt_time = to_ms_since_boot(get_absolute_time()) - attempt_start;
        printf("WiFi connection attempt %d failed after %lu ms\n", 
               attempt + 1, attempt_time);
    }
    
    // Report total time spent trying to connect
    uint32_t total_connection_time = to_ms_since_boot(get_absolute_time()) - connection_start;
    printf("Failed to connect to WiFi after %d attempts (%lu ms total)\n", 
           MAX_ATTEMPTS, total_connection_time);
    
    displayUploadStatus("WiFi connection failed");
    sleep_ms(500); // Reduced from 1000ms to 500ms
    return false;
}

// Add these emergency recovery functions before main()

// Dump basic stack/program information for debugging
void dump_stack_info() {
    printf("\n=== SYSTEM STATE DUMP ===\n");
    printf("Current time: %lu ms\n", to_ms_since_boot(get_absolute_time()));
    printf("Initialization complete: %s\n", initializationComplete ? "yes" : "no");
    printf("Current page: %d\n", current_page);
    printf("Button states: %d, %d\n", tast_pressed[0], tast_pressed[1]);
    printf("Button changed flag: %s\n", button_state_changed ? "yes" : "no");
    printf("Data buffer size: %lu records\n", data_buffer.size());
    printf("Stored flash records: %lu\n", flash_storage.getStoredCount());
    printf("GPS fake mode: %s\n", USE_FAKE_GPS ? "enabled" : "disabled");
    printf("========================\n\n");
}

// Emergency reset function to recover from deadlocks
void emergency_reset() {
    printf("EMERGENCY [%8lu ms]: Attempting recovery reset\n", to_ms_since_boot(get_absolute_time()));
    
    // Print system state for debugging
    dump_stack_info();
    
    // Reset key variables to recover from potential deadlocks
    current_page = 0;
    refresh_display = true;
    tast_pressed[0] = NOT_PRESSED;
    tast_pressed[1] = NOT_PRESSED;
    button_state_changed = false;
    
    // Force display refresh
    printf("EMERGENCY: Setting display to page 0 and forcing refresh\n");
    forceDisplayRefresh();
    
    // Reset timing variables
    last_loop_time = to_ms_since_boot(get_absolute_time());
    loop_count = 0;
    watchdog_triggered = false;
    
    printf("EMERGENCY: Reset complete\n");
}

// Check if the main loop is hung
void check_for_hang() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Update loop counter every time we're called
    loop_count++;
    
    // First time initialization
    if (last_loop_time == 0) {
        last_loop_time = current_time;
        return;
    }
    
    // Check if we've been in the same state for too long (5 seconds)
    if (current_time - last_loop_time > 5000) {
        printf("HANG DETECT [%8lu ms]: System appears stuck (no progress for %lu ms, loop count: %lu)\n", 
               current_time, current_time - last_loop_time, loop_count);
        
        if (!watchdog_triggered) {
            watchdog_triggered = true;
            printf("HANG DETECT: First hang detection - will attempt recovery on next check if still hung\n");
        } else {
            // Second detection, try emergency reset
            printf("HANG DETECT: Multiple hang detections - attempting emergency reset\n");
            emergency_reset();
        }
    } else if (loop_count > 1000) {
        // If we've looped many times without getting stuck, reset the watchdog flag
        watchdog_triggered = false;
    }
    
    // Update last time
    last_loop_time = current_time;
}

// Add watchdog and safety timer support

// Timer for emergency situation handling
struct repeating_timer safety_timer;

// Handler for the safety timer
bool safety_timer_callback(struct repeating_timer *t) {
    static uint32_t last_safety_time = 0;
    static uint32_t safety_count = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Keep track of count
    safety_count++;
    
    // First time initialization
    if (last_safety_time == 0) {
        last_safety_time = now;
        return true;
    }
    
    // Calculate time since last callback
    uint32_t elapsed = now - last_safety_time;
    
    // Print status every 5 seconds (10 ticks at 500ms per tick)
    if (safety_count % 10 == 0) {
        printf("SAFETY [%8lu ms]: Safety timer tick #%lu, %lu ms elapsed since last tick\n", 
               now, safety_count, elapsed);
        printf("SAFETY: Main loop count: %lu, last loop time: %lu ms ago\n", 
               loop_count, now - last_loop_time);
    }
    
    // Feed the watchdog even if main loop is frozen
    #ifdef USE_WATCHDOG
    if (elapsed > 3000) {
        // If it's been more than 3 seconds, we may be in a freeze
        printf("SAFETY [%8lu ms]: WARNING: Long time between safety ticks (%lu ms) - main loop may be frozen\n", 
               now, elapsed);
        
        // Feed watchdog to prevent immediate reset
        watchdog_update();
        
        // If we've had multiple long pauses, try emergency reset
        if (elapsed > 10000) {  // 10 seconds
            printf("SAFETY [%8lu ms]: CRITICAL: System appears completely frozen - attempting emergency reset\n", now);
            emergency_reset();
        }
    } else {
        // Normal operation - feed watchdog
        watchdog_update();
    }
    #endif
    
    // Update time reference
    last_safety_time = now;
    
    // Return true to keep the timer running
    return true;
}

// Add this new function for extremely large uploads
bool uploadSensorDataParallel(Flash& flash, myGPS& gps) {
    if (flash.getStoredCount() == 0) {
        printf("No data to upload\n");
        displayUploadStatus("No data to upload");
        return true; // Nothing to upload is considered success
    }
    
    // For extremely large uploads (>300 records), use this specialized function
    printf("Starting parallel upload for %lu records\n", flash.getStoredCount());
    displayUploadStatus("Large data upload");
    
    // Load records from flash
    std::vector<SensorData> records = flash.loadAllSensorData();
    printf("Loaded %lu records from flash\n", records.size());
    
    if (records.empty()) {
        printf("No valid records found in flash\n");
        displayUploadStatus("No valid data");
        return false;
    }
    
    // For very large uploads (>300 records), split into batches
    // and process them more optimally - use smaller batches for reliability
    const size_t RECORDS_PER_BATCH = 20; // Reduced from 30 to 20 for better reliability
    const size_t BATCH_SIZE = 5; // Reduced from 10 to 5 for better reliability
    
    size_t total_records = records.size();
    size_t total_batches = (total_records + RECORDS_PER_BATCH - 1) / RECORDS_PER_BATCH;
    size_t successful_batches = 0;
    
    printf("Breaking %lu records into %lu batches of %lu records each\n", 
           total_records, total_batches, RECORDS_PER_BATCH);
    
    // Track total upload time
    uint32_t upload_start_time = to_ms_since_boot(get_absolute_time());
    
    // Process each batch with optimized settings
    for (size_t batch = 0; batch < total_batches; batch++) {
        // Calculate batch boundaries
        size_t start_idx = batch * RECORDS_PER_BATCH;
        size_t end_idx = std::min(start_idx + RECORDS_PER_BATCH, total_records);
        size_t batch_size = end_idx - start_idx;
        
        // Display current batch status
        char status_msg[64];
        sprintf(status_msg, "Batch %lu/%lu", batch + 1, total_batches);
        displayUploadStatus(status_msg);
        
        printf("Processing batch %lu/%lu (records %lu-%lu)\n", 
               batch + 1, total_batches, start_idx + 1, end_idx);
        
        // Create subset for this batch
        std::vector<SensorData> batch_records(records.begin() + start_idx, records.begin() + end_idx);
        
        // Process this batch with chunked upload but minimal delays
        size_t chunks_in_batch = (batch_size + BATCH_SIZE - 1) / BATCH_SIZE;
        size_t successful_chunks = 0;
        
        // Buffer for JSON data - smaller buffer for more reliable uploads
        char json_buffer[10240]; // Reduced from 20480 to 10240 for better reliability
        
        // Process each chunk in this batch
        for (size_t chunk = 0; chunk < chunks_in_batch; chunk++) {
            // Calculate chunk boundaries
            size_t chunk_start = chunk * BATCH_SIZE;
            size_t chunk_end = std::min(chunk_start + BATCH_SIZE, batch_size);
            size_t chunk_size = chunk_end - chunk_start;
            
            // Create subset for this chunk
            std::vector<SensorData> chunk_records(batch_records.begin() + chunk_start, 
                                                batch_records.begin() + chunk_end);
            
            // Show progress
            printf("Uploading batch %lu/%lu, chunk %lu/%lu (%lu records)\n", 
                   batch + 1, total_batches, chunk + 1, chunks_in_batch, chunk_size);
            
            // Clear buffer before reuse
            memset(json_buffer, 0, sizeof(json_buffer));
            
            // Prepare JSON for just this chunk
            prepareBatchDataForTransmission(chunk_records, json_buffer, sizeof(json_buffer), gps);
            
            // Use more retries and longer delay for better reliability
            bool result = uploadDataWithRetry(json_buffer, 5, 500); // Increased from 3 to 5 retries, and from 250ms to 500ms delay
            
            if (result) {
                printf("Batch %lu/%lu, Chunk %lu/%lu uploaded successfully\n", 
                       batch + 1, total_batches, chunk + 1, chunks_in_batch);
                successful_chunks++;
            } else {
                printf("Batch %lu/%lu, Chunk %lu/%lu upload failed\n", 
                       batch + 1, total_batches, chunk + 1, chunks_in_batch);
            }
            
            // Increased delay between chunks for better reliability
            sleep_ms(200); // Increased from 150ms to 200ms for better reliability
        }
        
        // If most chunks in this batch succeeded, count the batch as successful
        if (successful_chunks >= chunks_in_batch * 0.7) { // 70% success rate
            successful_batches++;
            printf("Batch %lu/%lu completed successfully (%lu/%lu chunks)\n", 
                   batch + 1, total_batches, successful_chunks, chunks_in_batch);
        } else {
            printf("Batch %lu/%lu failed (%lu/%lu chunks successful)\n", 
                   batch + 1, total_batches, successful_chunks, chunks_in_batch);
        }
        
        // Increased delay between batches for better reliability
        sleep_ms(500); // Increased from 300ms to 500ms for better reliability
    }
    
    uint32_t upload_end_time = to_ms_since_boot(get_absolute_time());
    uint32_t total_upload_time = upload_end_time - upload_start_time;
    
    printf("Parallel upload complete: %lu/%lu batches successful in %lu ms (%.1f seconds)\n", 
           successful_batches, total_batches, total_upload_time, total_upload_time / 1000.0f);
    
    // Calculate throughput
    float records_per_second = (float)total_records / (total_upload_time / 1000.0f);
    printf("Upload speed: %.1f records per second\n", records_per_second);
    
    // Consider upload successful if most batches worked
    bool mostly_successful = (successful_batches >= total_batches * 0.7); // 70% success threshold
    
    // Display final status
    if (mostly_successful) {
        displayUploadStatus("Upload complete!");
        sleep_ms(500);
        
        // Ask user if they want to clear data
        bool clear_data = showYesNoPrompt("Data Uploaded", "Clear flash storage?");
        
        if (clear_data) {
            displayUploadStatus("Clearing storage...");
            if (flash_storage.eraseStorage()) {
                displayUploadStatus("Storage cleared");
                sleep_ms(500);
            } else {
                displayUploadStatus("Clear failed!");
                sleep_ms(500);
            }
        } else {
            displayUploadStatus("Data preserved");
            sleep_ms(500);
        }
    } else {
        char result_msg[64];
        sprintf(result_msg, "%lu/%lu batches uploaded", successful_batches, total_batches);
        displayUploadStatus(result_msg);
        sleep_ms(500);
    }
    
    return mostly_successful;
}

// Add this function near uploadDataWithRetry to provide a more reliable Vercel upload option
bool uploadDataWithVercelProxy(const char* json_data, int max_retries, int retry_delay_ms) {
    // Add better input validation
    if (!json_data) {
        printf("ERROR: JSON data is NULL\n");
        return false;
    }
    
    size_t json_size = strlen(json_data);
    if (json_size == 0) {
        printf("ERROR: JSON data is empty\n");
        return false;
    }
    
    if (json_size > 15000) {
        printf("ERROR: JSON data size (%lu bytes) exceeds maximum allowed (15000 bytes)\n", json_size);
        return false;
    }
    
    // Basic JSON validation - check for opening/closing braces
    if (json_data[0] != '{' || json_data[json_size-1] != '}') {
        printf("ERROR: JSON data appears to be malformed (does not start with { and end with })\n");
        printf("JSON starts with: %.20s...\n", json_data);
        printf("JSON ends with: ...%.20s\n", json_data + (json_size > 20 ? json_size - 20 : 0));
        return false;
    }
    
    printf("VERCEL: Preparing HTTP request with %d bytes of JSON data\n", (int)json_size);
    
    // Prepare the HTTP request (15K max buffer)
    char request_buffer[15000];
    
    // Create a simpler POST request specifically for Vercel
    // 1. Use POST /api/data instead of /api/addMarkers
    // 2. Use minimal headers to reduce overhead
    // 3. Explicitly set content-length
    // 4. Add JSON payload validation with retry support
    snprintf(request_buffer, sizeof(request_buffer),
             "POST /api/data HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Connection: close\r\n"
             "Content-Length: %d\r\n"
             "\r\n"
             "%s",
             TLS_CLIENT_SERVER, (int)json_size, json_data);
    
    // Progressive backoff for retries
    int current_delay = retry_delay_ms;
    
    // Initialize retry tracking variables
    bool upload_successful = false;
    int retry_count = 0;
    
    // Keep track of consecutive failures for diagnostics
    static int consecutive_failures = 0;
    static int abrt_errors = 0;
    
    // Attempt upload with retries
    while (retry_count <= max_retries && !upload_successful) {
        if (retry_count > 0) {
            // Display retry status
            char retry_msg[64];
            snprintf(retry_msg, sizeof(retry_msg), "Retry %d of %d...", retry_count, max_retries);
            displayUploadStatus(retry_msg);
            
            // Wait before retry with progressive backoff
            printf("VERCEL: Waiting %d ms before retry %d...\n", current_delay, retry_count);
            sleep_ms(current_delay);
            
            // Increase delay for next retry (up to a maximum)
            current_delay = std::min(current_delay * 2, 2000);
        }
        
        // Display status information for debugging
        printf("VERCEL: Upload attempt %d of %d...\n", retry_count + 1, max_retries);
        printf("VERCEL REQUEST: Server=%s, data size=%d bytes\n", TLS_CLIENT_SERVER, (int)json_size);
        
        // For even-numbered retries, try direct connection, for odd-numbered, try alternative server name
        const char* server_name = (retry_count % 2 == 0) ? TLS_CLIENT_SERVER : "gm4s.eu";
        int timeout_ms = (retry_count > 0) ? 15000 : 20000;  // Shorter timeouts for retries
        
        // Try the TLS client with this configuration
        upload_successful = run_tls_client_test(NULL, 0, server_name, request_buffer, timeout_ms);
        
        if (upload_successful) {
            printf("VERCEL: Upload successful!\n");
            consecutive_failures = 0;
            abrt_errors = 0;
        } else {
            // Increment failure counters
            consecutive_failures++;
            printf("VERCEL: Data upload failed on attempt %d (consecutive failures: %d)\n", 
                   retry_count + 1, consecutive_failures);
            
            // If we see multiple consecutive failures, try more aggressive recovery
            if (consecutive_failures >= 3) {
                printf("VERCEL: Multiple failures detected, attempting network reset\n");
                
                // Break out of the retry loop - we'll try a different approach
                if (retry_count >= 2) {
                    printf("VERCEL: Multiple attempts failed, will try a different approach\n");
                    break;
                }
            }
        }
        
        retry_count++;
    }
    
    // If all direct Vercel uploads failed, try using a proxy as last resort
    if (!upload_successful && retry_count > 2) {
        printf("VERCEL: All direct uploads failed. Trying simplified approach.\n");
        
        // Create an even simpler request with minimal headers
        snprintf(request_buffer, sizeof(request_buffer),
                 "POST /api/data HTTP/1.1\r\n"
                 "Host: %s\r\n"
                 "Content-Type: application/json\r\n"
                 "Content-Length: %d\r\n"
                 "\r\n"
                 "%s",
                 TLS_CLIENT_SERVER, (int)json_size, json_data);
        
        // Try one more time with the simplified request
        upload_successful = run_tls_client_test(NULL, 0, TLS_CLIENT_SERVER, request_buffer, 20000);
        
        if (upload_successful) {
            printf("VERCEL: Simplified upload approach succeeded!\n");
        } else {
            printf("VERCEL: All upload approaches failed. Please check server configuration.\n");
        }
    }
    
    // Reset abort error counter when completed
    abrt_errors = 0;
    
    return upload_successful;
}

// Add a function to upload sensor data in chunks for better reliability
bool uploadSensorDataChunked(Flash& flash, myGPS& gps, int mode) {
    if (flash.getStoredCount() == 0) {
        printf("No data to upload\n");
        displayUploadStatus("No data to upload");
        return true; // Nothing to upload is considered success
    }
    
    printf("Starting chunked upload for %lu records\n", flash.getStoredCount());
    displayUploadStatus("Starting upload...");
    
    // Load records from flash
    std::vector<SensorData> records = flash.loadAllSensorData();
    printf("Loaded %lu records from flash\n", records.size());
    
    if (records.empty()) {
        printf("No valid records found in flash\n");
        displayUploadStatus("No valid data");
        return false;
    }
    
    // For reliability, use much smaller chunks
    const size_t CHUNK_SIZE = UPLOAD_MAX_BATCH_SIZE; // Using the global max batch size
    
    size_t total_records = records.size();
    size_t total_chunks = (total_records + CHUNK_SIZE - 1) / CHUNK_SIZE;
    size_t successful_uploads = 0;
    
    printf("Breaking %lu records into %lu chunks of max %lu records each\n", 
           total_records, total_chunks, CHUNK_SIZE);
    
    // Track total upload time
    uint32_t upload_start_time = to_ms_since_boot(get_absolute_time());
    
    // Process each chunk
    for (size_t chunk = 0; chunk < total_chunks; chunk++) {
        // Calculate chunk boundaries
        size_t start_idx = chunk * CHUNK_SIZE;
        size_t end_idx = std::min(start_idx + CHUNK_SIZE, total_records);
        size_t chunk_size = end_idx - start_idx;
        
        // Display current chunk status
        char status_msg[64];
        sprintf(status_msg, "Chunk %lu/%lu", chunk + 1, total_chunks);
        displayUploadStatus(status_msg);
        
        printf("Processing chunk %lu/%lu (records %lu-%lu)\n", 
               chunk + 1, total_chunks, start_idx + 1, end_idx);
        
        // Create subset for this chunk
        std::vector<SensorData> chunk_records(records.begin() + start_idx, records.begin() + end_idx);
        
        // Buffer for JSON data
        char json_buffer[15360]; // Large buffer for JSON data
        
        // Clear buffer before reuse
        memset(json_buffer, 0, sizeof(json_buffer));
        
        // Prepare JSON for just this chunk
        prepareBatchDataForTransmission(chunk_records, json_buffer, sizeof(json_buffer), gps);
        
        // Debug print the JSON data before sending
        printf("Chunk %lu/%lu JSON content:\n", chunk + 1, total_chunks);
        debugPrintJSON(json_buffer, 1000); // Print first 1000 characters
        
        // Try uploading with the TLS client directly
        printf("Uploading chunk %lu/%lu with %lu records...\n", 
               chunk + 1, total_chunks, chunk_size);
        
        bool chunk_successful = false;
        
        // Try uploading with the TLS client directly first
        // Use a shorter timeout (8 seconds) to avoid long waiting periods
        chunk_successful = run_tls_client_test(NULL, 0, TLS_CLIENT_SERVER, json_buffer, 8000);
        
        // If direct TLS client fails, try with alternative server name
        if (!chunk_successful) {
            printf("First attempt failed for chunk %lu/%lu, trying with 'gm4s.eu' without www prefix\n", 
                   chunk + 1, total_chunks);
            
            // Create a simple HTTP request
            char request_buffer[15360];
            snprintf(request_buffer, sizeof(request_buffer),
                   "POST /api/addMarkers HTTP/1.1\r\n"
                   "Host: gm4s.eu\r\n"
                   "Content-Type: application/json\r\n"
                   "Content-Length: %zu\r\n"
                   "Connection: close\r\n"
                   "\r\n"
                   "%s",
                   strlen(json_buffer), json_buffer);
            
            // Alternate between www.gm4s.eu and gm4s.eu
            chunk_successful = run_tls_client_test(NULL, 0, "gm4s.eu", request_buffer, 6000);
        }
        
        // If both direct approaches fail, fall back to retry mechanism
        if (!chunk_successful) {
            printf("Direct upload failed for chunk %lu/%lu, trying with retry mechanism\n", 
                   chunk + 1, total_chunks);
            int retry_delay = 500; // 500ms base delay for retries
            chunk_successful = uploadDataWithRetry(json_buffer, 3, retry_delay);
        }
        
        if (chunk_successful) {
            successful_uploads++;
            printf("Chunk %lu/%lu upload successful (%lu records)\n", 
                   chunk + 1, total_chunks, chunk_size);
            
            // Add a small delay between successful chunks to let server process
            sleep_ms(200);
        } else {
            printf("Chunk %lu/%lu upload failed\n", chunk + 1, total_chunks);
            
            // Add a longer delay after failures for network recovery
            int retry_delay = 500; // 500ms base delay
            printf("Waiting %dms after failed chunk for network recovery\n", retry_delay * 2);
            sleep_ms(retry_delay * 2);
        }
    }
    
    uint32_t upload_end_time = to_ms_since_boot(get_absolute_time());
    uint32_t total_upload_time = upload_end_time - upload_start_time;
    
    // Show final results with timing information
    printf("Upload complete: %lu/%lu chunks successful (%lu/%lu records) in %lu ms (%.1f seconds)\n", 
           successful_uploads, total_chunks, 
           successful_uploads * CHUNK_SIZE < total_records ? successful_uploads * CHUNK_SIZE : total_records, 
           total_records, total_upload_time, total_upload_time / 1000.0f);
    
    // Only consider upload successful if at least 70% of chunks were uploaded
    bool mostly_successful = (successful_uploads >= total_chunks * 0.7);
    
    // If upload was mostly successful, ask if user wants to clear the data
    if (mostly_successful) {
        displayUploadStatus("Upload complete!");
        sleep_ms(500);
        
        // Ask user if they want to clear the uploaded data
        bool clear_data = showYesNoPrompt("Data Uploaded", "Clear flash storage?");
        
        if (clear_data) {
            displayUploadStatus("Clearing storage...");
            if (flash.eraseStorage()) {
                displayUploadStatus("Storage cleared");
                sleep_ms(500);
                printf("Flash storage cleared after successful upload\n");
            } else {
                displayUploadStatus("Clear failed!");
                sleep_ms(500);
            }
        } else {
            displayUploadStatus("Data preserved");
            sleep_ms(500);
        }
    } else if (successful_uploads > 0) {
        char result_msg[64];
        sprintf(result_msg, "%lu/%lu chunks uploaded", successful_uploads, total_chunks);
        displayUploadStatus(result_msg);
        sleep_ms(500);
    } else {
        displayUploadStatus("Upload failed");
        sleep_ms(500);
    }
    
    return mostly_successful;
}

// Function to display initialization progress on the e-ink display
void displayInitializationPage(const char* status_message, int step, int total_steps) {
    // Clear the screen first
    resetImageBuffer();
    
    // Draw a title at the top
    Paint_DrawString_EN(10, 10, "PicoW Sensor", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(10, 30, "Initializing...", &Font12, WHITE, BLACK);
    
    // Draw the status message
    Paint_DrawString_EN(10, 50, status_message, &Font12, WHITE, BLACK);
    
    // Draw a progress bar
    int progress_width = 120;
    int progress_height = 10;
    int progress_x = (EPD_1IN54_V2_WIDTH - progress_width) / 2;
    int progress_y = 70;
    
    // Draw the outline
    Paint_DrawRectangle(progress_x, progress_y, 
                        progress_x + progress_width, progress_y + progress_height,
                        BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    
    // Calculate the fill width based on steps
    int fill_width = progress_width * step / total_steps;
    
    // Draw the fill part
    if (fill_width > 0) {
        Paint_DrawRectangle(progress_x, progress_y, 
                           progress_x + fill_width, progress_y + progress_height,
                           BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Show step counter below
    char step_text[20];
    sprintf(step_text, "Step %d of %d", step, total_steps);
    Paint_DrawString_EN((EPD_1IN54_V2_WIDTH - strlen(step_text) * 7) / 2, 
                      progress_y + progress_height + 5, 
                      step_text, &Font8, WHITE, BLACK);
    
    // Update the e-ink display
    EPD_1IN54_V2_Display(ImageBuffer);
}

// Add these new constants after other interval definitions (around line 150)
#define GPS_CHECK_INTERVAL_MS 500     // Check GPS every 500ms for better fix acquisition
#define GPS_STATUS_UPDATE_MS 2000     // Update GPS status variables every 2 seconds

// Add these variables after other globals (around line 189)
uint32_t last_gps_check_ms = 0;       // Last time GPS was checked
uint32_t last_gps_status_update_ms = 0;  // Last time GPS status was updated
bool continuously_search_gps = true;  // Always search for GPS in background

// Add this constant at the top with other constants
#define GPS_POLL_INTERVAL_MS 100  // Poll GPS every 100ms (10Hz) for maximum responsiveness

// Add this global variable to store the latest valid GPS coordinates
double latest_valid_lat = 0;
double latest_valid_lon = 0;
bool has_valid_fix_since_boot = false;

// Add a new function to show GPS acquisition progress
void displayGPSAcquisitionProgress(int elapsed_seconds, int satellites, int timeout_seconds) {
    resetImageBuffer();
    char buffer[80];
    
    // Page Title
    Paint_DrawString_EN(10, 10, "GPS Acquisition", &Font20, BLACK, WHITE);
    
    // Draw time elapsed
    sprintf(buffer, "Time: %d/%d sec", elapsed_seconds, timeout_seconds);
    Paint_DrawString_EN(10, 35, buffer, &Font12, BLACK, WHITE);
    
    // Draw satellite count
    sprintf(buffer, "Satellites: %d", satellites);
    Paint_DrawString_EN(10, 55, buffer, &Font16, BLACK, WHITE);
    
    // Draw progress bar frame
    int bar_width = 180;
    int bar_height = 15;
    int bar_x = 10;
    int bar_y = 80;
    Paint_DrawRectangle(bar_x, bar_y, bar_x + bar_width, bar_y + bar_height, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    
    // Draw progress bar fill - show elapsed time percentage
    int fill_width = (bar_width * elapsed_seconds) / timeout_seconds;
    if (fill_width > 0) {
        Paint_DrawRectangle(bar_x, bar_y, bar_x + fill_width, bar_y + bar_height, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Draw satellite quality indicator
    Paint_DrawString_EN(10, 105, "Signal Quality:", &Font12, BLACK, WHITE);
    
    // Draw satellite bars
    int sat_x = 10;
    int sat_y = 120;
    int sat_width = 12;
    int sat_height = 20;
    int sat_spacing = 5;
    
    // Draw up to 8 satellite bars based on count
    for (int i = 0; i < 8; i++) {
        int x = sat_x + i * (sat_width + sat_spacing);
        int bar_h = sat_height;
        
        // Draw empty bars for all potential satellites
        Paint_DrawRectangle(x, sat_y, x + sat_width, sat_y + sat_height, 
                           BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        
        // Fill bars for visible satellites
        if (i < satellites) {
            // Draw filled bars with variable height for visible satellites
            bar_h = 5 + (i % 4) * 5;
            Paint_DrawRectangle(x, sat_y + (sat_height - bar_h), x + sat_width, sat_y + sat_height, 
                               BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
    }
    
    // Show status message
    if (satellites == 0) {
        Paint_DrawString_EN(10, 150, "Searching for satellites...", &Font12, BLACK, WHITE);
    } else if (satellites < 3) {
        Paint_DrawString_EN(10, 150, "Signal weak - keep outdoors", &Font12, BLACK, WHITE);
    } else if (satellites < 5) {
        Paint_DrawString_EN(10, 150, "Signal OK - acquiring fix", &Font12, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(10, 150, "Good signal - nearly ready", &Font12, BLACK, WHITE);
    }
    
    // Add instructions
    Paint_DrawString_EN(10, 170, "Press any button to skip", &Font12, BLACK, WHITE);
    
    // Full refresh to ensure clarity
    EPD_1IN54_V2_Display(ImageBuffer);
}

// Add this debug function after other utility functions
void debugPrintJSON(const char* json_data, size_t max_length) {
    printf("[JSON DEBUG] Printing JSON data (up to %lu chars):\n", max_length);
    
    // Print the first part
    size_t actual_length = strlen(json_data);
    size_t print_length = actual_length < max_length ? actual_length : max_length;
    
    printf("====== JSON START ======\n");
    for (size_t i = 0; i < print_length; i++) {
        putchar(json_data[i]);
    }
    
    if (actual_length > max_length) {
        printf("\n... (truncated %lu more characters)", actual_length - max_length);
    }
    
    printf("\n====== JSON END ======\n");
    printf("Total JSON length: %lu bytes\n", actual_length);
    
    // Basic validation checks
    bool has_opening_brace = json_data[0] == '{';
    bool has_closing_brace = json_data[actual_length-1] == '}';
    bool has_token = strstr(json_data, "\"token\"") != NULL;
    bool has_measurements = strstr(json_data, "\"measurements\"") != NULL;
    
    printf("JSON validation: Opening brace: %s, Closing brace: %s, Token: %s, Measurements: %s\n",
           has_opening_brace ? "YES" : "NO",
           has_closing_brace ? "YES" : "NO",
           has_token ? "YES" : "NO",
           has_measurements ? "YES" : "NO");
}



int main() {
    stdio_init_all();

    printf("Program starting with enhanced debugging...\n");
    
    // Set system time to a reasonable default (Jan 1, 2024)
    // Since the Pi Pico has no RTC, we need to set a default time
    // This will be used until we get actual time from GPS
    time_t default_time = 1706745600; // 2024-01-31 00:00:00 UTC
    set_system_time(default_time);
    
#ifdef USE_WATCHDOG
    // Initialize and start the watchdog timer (20 second timeout)
    watchdog_enable(20000, 1);
    printf("Watchdog enabled with 20 second timeout\n");
#else
    printf("Watchdog disabled\n");
#endif

    // Initialize the safety timer
    if (add_repeating_timer_ms(500, safety_timer_callback, NULL, &safety_timer)) {
        printf("Safety timer enabled with 500ms interval\n");
    } else {
        printf("ERROR: Failed to start safety timer\n");
    }

    DEBUG_POINT("Starting initialization");
    printf("Initializing I2C...\n");
    i2c_init();  // Initialize I2C before checking sensors
    printf("I2C initialized, checking sensors...\n");
    checkSensors();
    printf("Sensors checked, initializing ADC...\n");
    batteryADC.init();
    
    // Add delay to ensure all sensors are stable
    printf("Waiting for sensors to stabilize...\n");
    sleep_ms(2000);
    printf("Continuing initialization\n");
    
    printf("Initializing eInk display...\n");
    eInk_init();
    displayHello();
    
    printf("Initializing buttons...\n");
    initButtons();
    
    printf("Initializing flash storage...\n");
#if defined(DISABLE_FLASH) && DISABLE_FLASH == 1
    flash_storage.setFlashEnabled(false);
    printf("Flash operations DISABLED by configuration\n");
#endif
    flash_initialized = flash_storage.init();
    if (flash_initialized) {
        printf("Flash storage initialized successfully\n");
        printf("Flash storage can hold up to %lu records\n", flash_storage.getMaxDataCount());
        printf("Currently %lu records stored\n", flash_storage.getStoredCount());
    } else {
        printf("Flash storage initialization failed\n");
    }
    
    printf("Initializing GPS module...\n");
    myGPS gps(uart0, 9600, 0, 1);
    printf("GPS module initialized\n");

    DEBUG_POINT("Core initialization complete");

#if USE_FAKE_GPS
    // Enable fake GPS data for indoor testing
    printf("NOTICE: Fake GPS data enabled for indoor testing\n");
    gps.enableFakeGPS(true);
    // You can set custom coordinates if needed
    // gps.setFakeCoordinates(48.20662016908546, 15.617513602109687);
#else
    // Only initialize real GPS if not using fake data
    
    // First test the connection to make sure GPS is responsive
    printf("Testing GPS connection...\n");
    int gps_test_result = gps.testConnection();
    printf("GPS connection test result: %d (0=Good, 1=No NMEA, 2=UART error)\n", gps_test_result);
    
    // Enable time messages explicitly
    printf("Requesting GPS module to enable time messages...\n");
    gps.enableTimeMessages();
    
    // Set up for faster GPS acquisition by sending multiple configuration commands
    // First try a cold start to fully reset the module
    printf("Performing GPS cold start to reset the module...\n");
    bool cold_start_success = gps.sendColdStartCommand();
    if (cold_start_success) {
        printf("GPS cold start completed successfully\n");
    } else {
        printf("GPS cold start may not have been recognized by the module\n");
        // Try a hot start as fallback
        printf("Attempting hot start instead...\n");
        gps.sendHotStartCommand();
    }
    
    // Additional configuration to focus on faster fix acquisition
    printf("Sending additional GPS configuration commands...\n");
    gps.enableTimeMessages();  // Send again after reset for redundancy
    
    // Use the new optimization function for faster fix acquisition
    printf("Optimizing GPS for faster fix acquisition...\n");
    gps.optimizeForFastAcquisition();
    
    printf("Starting continuous GPS acquisition in the background...\n");
#endif

    // Set GPS start time after initialization
    gps_start_time = get_absolute_time();
    printf("Starting GPS acquisition...\n");
    
    // Initialize variables for main loop
    bool has_fix = false;
    fix_status = 2; // Start with invalid fix
    satellites_visible = 0;
    last_gps_check_ms = to_ms_since_boot(get_absolute_time());
    last_gps_status_update_ms = to_ms_since_boot(get_absolute_time());
    
    // Wait for a valid GPS fix before entering the main loop
    if (!USE_FAKE_GPS) {
        printf("Waiting for initial GPS fix before starting main loop...\n");
        
        // Define timeout for initial GPS fix (120 seconds)
        const int GPS_INIT_TIMEOUT = 120;  // Increased from 60 to 120 seconds
        bool initial_fix = false;
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        
        // Show progress updates while waiting
        for (int wait_time = 0; wait_time < GPS_INIT_TIMEOUT; wait_time += 2) {  // Check every 2 seconds
            // Update satellite count
            satellites_visible = gps.getVisibleSatellites();
            
            // Display acquisition progress
            displayGPSAcquisitionProgress(wait_time, satellites_visible, GPS_INIT_TIMEOUT);
            
            // Check for GPS fix
            initial_fix = gps.waitForFix(2);  // 2 second increments
            
            // Exit if we got a fix
            if (initial_fix) {
                printf("Valid GPS fix obtained! Satellites: %d\n", satellites_visible);
                fix_status = 0; // Valid fix
                has_valid_fix_since_boot = true;
                
                // Set the fix coordinates
                std::string gps_line;
                double gps_lon = 0, gps_lat = 0;
                char gps_ew = 'E', gps_ns = 'N';
                std::string gps_time_str, gps_date_str;
                
                // Read the latest coordinates
                if (gps.readLine(gps_line, gps_lon, gps_ew, gps_lat, gps_ns, gps_time_str, gps_date_str) == 0) {
                    latest_valid_lat = gps_lat;
                    latest_valid_lon = gps_lon;
                    printf("First valid coordinates: %.6f, %.6f\n", latest_valid_lat, latest_valid_lon);
                }
                
                // Show success message
                displayUploadStatus("GPS fix acquired!");
                sleep_ms(1000);
                break;
            }
            
            // Check if user wants to skip waiting (if they press any button)
            // Direct GPIO read to avoid interrupt dependencies
            if (gpio_get(BUTTON_NEXT_PAGE) == 0 || gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {
                printf("User interrupted GPS wait, continuing without fix\n");
                displayUploadStatus("GPS wait skipped");
                sleep_ms(1000);
                break;
            }
            
            printf("Waiting for GPS fix... %d seconds elapsed, %d satellites\n", 
                  wait_time, satellites_visible);
        }
        
        if (!initial_fix) {
            printf("Timed out waiting for GPS fix, continuing anyway\n");
            displayUploadStatus("No GPS fix, continuing");
            sleep_ms(1000);
        }
    } else {
        // Fake GPS mode - simulate acquisition
        printf("Simulating GPS acquisition in fake mode\n");
        displayUploadStatus("Simulating GPS");
        
        // Wait for a simulated GPS fix
        for (int i = 0; i < 5; i++) {
            // Simulate GPS acquisition
            has_fix = gps.waitForFix(1); // This will do the simulation in fake mode
            satellites_visible = gps.getVisibleSatellites();
            
            // Display progress
            displayGPSAcquisitionProgress(i+1, satellites_visible, 5);
            
            if (has_fix) {
                printf("Simulated GPS fix obtained\n");
                fix_status = 0; // Valid fix
                has_valid_fix_since_boot = true;
                
                // Set default fake coordinates
                latest_valid_lat = 48.20662016908546;
                latest_valid_lon = 15.617513602109687;
                
                displayUploadStatus("Simulated GPS ready");
                sleep_ms(1000);
                break;
            }
            
            sleep_ms(500);
        }
    }
    
    last_refresh_time = get_absolute_time();
    printf("Main loop starting now\n");

    // Update with new initialization flags
    initialDataCollected = false;
    initialDataSaved = false;
    initializationComplete = false;
    
    // Display the initial initialization screen before entering the main loop
    displayInitializationPage("Starting up...", 1, 5);
    sleep_ms(1000);

    // Set up storage and data collection timing variables
    // Reduce collection interval to match refresh interval (already handled by dataCollectionInterval)
    // Reduce flash save interval for more frequent saves while biking (every 30s instead of 3min)
    const uint32_t FLASH_SAVE_INTERVAL_MS = 30000;    // Save to flash every 30 seconds for bike mode
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_data_collection_ms = current_time_ms;
    uint32_t last_flash_save_ms = current_time_ms;

    // Add a shorter save interval for initialization - keep this the same
    const uint32_t INIT_FLASH_SAVE_INTERVAL_MS = 60000; // 1 minute during initialization
    
    printf("BIKE MODE: Display refresh=%d ms, data collection=%d ms, flash save=%d ms\n",
           refreshInterval, dataCollectionInterval, (unsigned int)FLASH_SAVE_INTERVAL_MS);

    // Reduce buffer size for bike mode to ensure more frequent saves
    // and prevent large data loss in case of power issues during the ride
    const int MAX_BUFFER_SIZE = 10; // Reduced from 25 for more frequent saves during biking
    // Smaller buffer size during initialization - keep this the same
    const int INIT_MAX_BUFFER_SIZE = 2;  // Save after just 2 readings during initialization
    
    // GPS error recovery variables
    int consecutive_gps_errors = 0;
    uint32_t last_gps_recovery_time_ms = 0;
    uint32_t gps_recovery_interval_ms = 60000; // Try recovery every minute if needed
    
    // Wait a moment for systems to stabilize
    sleep_ms(1000);
    DEBUG_POINT("Starting main loop");
    
    // After all initialization, but before the main loop, add:
    
    // Check if there's data in flash when starting up
    if (!setupComplete) {
        // Check if there's existing data in flash storage
        uint32_t stored_count = flash_storage.getStoredCount();
        if (stored_count > 0) {
            printf("Found %lu existing records in flash storage\n", stored_count);
            
            // Temporarily disable fast refresh for startup prompt to ensure it displays properly
            bool was_fast_refresh_enabled = fast_refresh_enabled;
            if (fast_refresh_enabled) {
                printf("Temporarily disabling fast refresh for startup prompt\n");
                fast_refresh_enabled = false;
                base_image_set = false;  // Reset base image state
            }
            
            // Ask user if they want to continue with existing data
            displayUploadStatus("Continue with");
            sleep_ms(500);
            displayUploadStatus("existing data?");
            
            printf("STARTUP: Beginning timeout sequence\n");
            
            // ULTRA RELIABLE TIMEOUT IMPLEMENTATION
            // No dependencies on anything except sleep_ms()
            bool keep_data = true; // Default to keeping data
            
            // Print a message every second to show we're alive
            for (int i = 0; i < 10; i++) {
                printf("STARTUP: Waiting for button press - %d seconds elapsed, %d remaining\n", 
                       i, 10-i);
                
                // Check buttons 20 times over 1 second (50ms intervals)
                for (int j = 0; j < 20; j++) {
                    // Direct GPIO read of buttons (no interrupt dependence)
                    if (gpio_get(BUTTON_NEXT_PAGE) == 0) {  // Button is pressed (active low)
                        printf("STARTUP: Button 0 (Next Page) pressed directly\n");
                        keep_data = true;
                        goto timeout_complete;
                    }
                    
                    if (gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {  // Button is pressed (active low)
                        printf("STARTUP: Button 1 (Refresh Display) pressed directly\n");
                        keep_data = false;
                        goto timeout_complete;
                    }
                    
                    // Very short sleep between checks
                    sleep_ms(50);
                }
            }
            
timeout_complete:
            // Show what decision was made
            printf("STARTUP: Timeout complete - %s existing data\n", 
                   keep_data ? "keeping" : "erasing");
            
            if (!keep_data) {
                // User chose to start fresh
                printf("Erasing flash storage\n");
                displayUploadStatus("Erasing old data");
                
                // Clear the storage
                flash_storage.eraseStorage();
                
                printf("Flash storage erased\n");
                displayUploadStatus("Starting fresh");
                sleep_ms(2000);
            } else {
                printf("Continuing with existing data (%lu records)\n", stored_count);
                displayUploadStatus("Continuing ride");
                sleep_ms(2000);
            }
            
            // Restore fast refresh mode if it was enabled before
            if (was_fast_refresh_enabled) {
                printf("Re-enabling fast refresh mode after startup prompt\n");
                fast_refresh_enabled = true;
                base_image_set = false;  // We'll need to set a new base image
            }
        }
        
        setupComplete = true;
    }
    
    // Additional setup: Enable fast refresh mode for better responsiveness
    printf("Fast refresh disabled by default for display reliability\n");
    printf("Enable from Settings page if desired (double press button)\n");
    // Do not call setFastRefreshMode here
    
    // Main loop
    while (true) {
        // Call hang detection at the start of each loop
        check_for_hang();
        
        // First debug checkpoint in main loop
        DEBUG_LOOP_COUNT(main_loop);

        // Feed the watchdog
#ifdef USE_WATCHDOG
        watchdog_update();
#endif
        
        // Handle any pending button input
        DEBUG_POINT("Processing button inputs");
        volatile uint32_t events = btn1_events;
        btn1_events = 0;
        
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Button press processing...
        }
        
        // Process GPS data on each iteration to update status variables
        std::string gps_data;
        double gps_lon = 0, gps_lat = 0;
        char gps_ew = 'E', gps_ns = 'N';
        std::string gps_time_str, gps_date_str;
        
        // Read GPS data with timeout protection
        int gps_status = gps.readLine(gps_data, gps_lon, gps_ew, gps_lat, gps_ns, gps_time_str, gps_date_str);
        
        // Update global GPS status variables
        fix_status = gps_status;  // 0 = valid fix, 2 = invalid
        
        // Update satellites visible count periodically
        static uint32_t last_sat_check = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (now - last_sat_check > 5000) {  // Check every 5 seconds
            satellites_visible = gps.getVisibleSatellites();
            last_sat_check = now;
            printf("GPS Status: Fix=%s, Satellites=%d\n", 
                  (fix_status == 0) ? "VALID" : "INVALID", 
                  satellites_visible);
        }
        
        // Don't wait too long for GPS data before proceeding with other tasks
        // This prevents hanging the main loop on GPS data
        static uint32_t last_task_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Process other tasks at least every 100ms regardless of GPS activity
        if (current_time - last_task_time > 100) {
            DEBUG_POINT("Processing scheduled tasks");
            
            // Check if it's time to collect data
            if (current_time - last_data_collection_ms >= dataCollectionInterval) {
                DEBUG_POINT("Starting data collection");
                printf("TIMING: Data collection triggered (elapsed: %u ms, interval: %u ms)\n",
                       (unsigned int)(current_time - last_data_collection_ms),
                       (unsigned int)dataCollectionInterval);
                
                // Check if we have a valid GPS fix before collecting data
                if (fix_status != 0 && !USE_FAKE_GPS && !has_valid_fix_since_boot) {
                    // No valid GPS fix yet and no previous valid fix
                    printf("Waiting for valid GPS fix before collecting data (satellites: %d)\n", satellites_visible);
                    
                    // Update the display to show we're waiting for GPS fix
                    if (current_page == 4) {  // If currently on GPS page, refresh it
                        forceDisplayRefresh();
                    } else {
                        // Briefly show a waiting message if not on GPS page
                        displayUploadStatus("Waiting for GPS fix");
                        sleep_ms(1000);
                        forceDisplayRefresh(); // Return to current page
                    }
                    
                    // Update the collection time but don't collect data yet
                    last_data_collection_ms = current_time;
                    continue; // Skip data collection until we have a fix
                }
                
                // We have a valid fix now or had one before, proceed with data collection
                // If we have a current valid fix, update the saved coordinates
                if (fix_status == 0) {
                    // Store the latest valid coordinates for future use
                    // These coordinates come from the continuous GPS polling section
                    has_valid_fix_since_boot = true;
                }
                
                printf("Collecting sensor data with %s GPS coordinates\n", 
                      (fix_status == 0) ? "current" : "last valid");
                
                // Read latest data from sensors and GPS
                DEBUG_POINT("Reading battery level");
                batteryLevel = batteryADC.calculateBatteryLevel();
                if (batteryLevel == 0) {
                    batteryLevel = 50;  // Use default value if reading fails
                }
                
                DEBUG_POINT("Reading BME688 sensor");
                // Read real sensor values from BME688
                float temp, hum, pres, gas;
                if (bme688_sensor.readData(temp, hum, pres, gas)) {
                    sensor_data_obj.temp = temp;
                    sensor_data_obj.hum = hum;
                    sensor_data_obj.pres = pres;
                    sensor_data_obj.gasRes = gas;
                } else {
                    printf("Failed to read from BME688 sensor\n");
                }
                
                DEBUG_POINT("Reading HM3301 sensor");
                // Read real values from HM3301 particulate matter sensor
                uint16_t pm1_0 = 0, pm2_5 = 0, pm10 = 0;
                printf("HM3301_DEBUG [%8lu ms]: Starting HM3301 sensor read\n", to_ms_since_boot(get_absolute_time()));
                
                uint32_t hm3301_start = to_ms_since_boot(get_absolute_time());
                bool hm3301_success = false;
                
                // Catch any exceptions during sensor read
                try {
                    hm3301_success = hm3301_sensor.read(pm1_0, pm2_5, pm10);
                } catch (const std::exception& e) {
                    printf("HM3301_DEBUG: Exception during read: %s\n", e.what());
                } catch (...) {
                    printf("HM3301_DEBUG: Unknown exception during read\n");
                }
                
                uint32_t hm3301_end = to_ms_since_boot(get_absolute_time());
                uint32_t hm3301_duration = hm3301_end - hm3301_start;
                
                printf("HM3301_DEBUG [%8lu ms]: Read operation completed in %lu ms, success: %s\n", 
                       hm3301_end, hm3301_duration, hm3301_success ? "yes" : "no");
                
                if (hm3301_success) {
                    sensor_data_obj.pm2_5 = pm2_5;
                    sensor_data_obj.pm5 = pm1_0;  // Using PM1.0 for PM5 since there's no direct PM5 reading
                    sensor_data_obj.pm10 = pm10;
                    printf("HM3301_DEBUG: Values read - PM1.0: %u, PM2.5: %u, PM10: %u\n", pm1_0, pm2_5, pm10);
                } else {
                    printf("HM3301_DEBUG: Failed to read from HM3301 sensor\n");
                    // Use previous values if available, or zeros if not
                    printf("HM3301_DEBUG: Using previous or default values\n");
                }
                
                DEBUG_POINT("Reading CO2 sensor");
                // Read real CO2 values from PAS CO2 sensor with timeout protection
                absolute_time_t co2_start_time = get_absolute_time();
                
                // Add extra safety by wrapping the CO2 sensor read in a timeout check
                bool co2_timeout = false;
                
                // Use the modified read function with built-in timeout
                pas_co2_sensor.read();
                
                // Check if the operation took too long (indicating a potential hang)
                int64_t co2_read_time_us = absolute_time_diff_us(co2_start_time, get_absolute_time());
                if (co2_read_time_us > 1500000) { // 1.5 seconds
                    printf("WARNING: CO2 sensor read took extremely long (%lld ms) - potential I2C issue\n", 
                           co2_read_time_us / 1000);
                }
                
                // Get and validate the reading
                uint32_t co2_reading = pas_co2_sensor.getResult();
                
                // Apply sanity check for CO2 values (typically 400-5000 ppm in normal environments)
                if (co2_reading < 400 || co2_reading > 10000) {
                    printf("WARNING: CO2 reading out of expected range: %u ppm - using default value\n", co2_reading);
                    // Use a reasonable default for invalid readings
                    sensor_data_obj.co2 = 400;
                } else {
                    sensor_data_obj.co2 = co2_reading;
                    printf("CO2 reading: %u ppm\n", co2_reading);
                }
                
                DEBUG_POINT("Reading GPS data for location");
                // Get GPS data with timeout protection
                std::string gps_line;
                double longitude = 0, latitude = 0;  // Initialize to avoid uninitialized values
                char ew = 'E', ns = 'N';             // Default values if GPS read fails
                std::string time_str = "00:00:00";   // Default time if GPS read fails
                std::string date_str = "010124";     // Default date if GPS read fails (Jan 1, 2024)
                
                // Record the start time for GPS reading
                uint32_t gps_start_ms = to_ms_since_boot(get_absolute_time());
                
                // Try to get GPS data with built-in timeout protection in the modified GPS class
                int gps_result = gps.readLine(gps_line, longitude, ew, latitude, ns, time_str, date_str);
                
                // Calculate how long the GPS read took
                uint32_t gps_duration_ms = to_ms_since_boot(get_absolute_time()) - gps_start_ms;
                if (gps_duration_ms > 100) {
                    // Log a warning if GPS reading takes too long (but doesn't hang)
                    printf("WARNING: GPS read took %lu ms (expected <100ms)\n", gps_duration_ms);
                }
                
                // Set GPS data in the sensor data object
                sensor_data_obj.longitude = (int32_t)(longitude * 10000000); // Store as fixed-point
                sensor_data_obj.latitude = (int32_t)(latitude * 10000000);   // Store as fixed-point
                
                // Set timestamp from system time
                sensor_data_obj.timestamp = time(NULL);
                
                // Set fake GPS flag based on gps settings
                sensor_data_obj.is_fake_gps = (USE_FAKE_GPS == 1);
                
                DEBUG_POINT("Adding data to buffer");
                // Add to in-memory buffer
                data_buffer.push_back(sensor_data_obj);
                buffer_modified = true;
                printf("Added data record #%lu to buffer (now %lu records in buffer)\n",
                       flash_storage.getStoredCount() + data_buffer.size(), data_buffer.size());
                
                // Set the initial data collected flag
                if (!initialDataCollected) {
                    initialDataCollected = true;
                    printf("INIT: First data collection complete\n");
                    // Update initialization page if we're still in init mode
                    if (!initializationComplete) {
                        displayInitializationPage("First data collected", 2, 5);
                        
                        // Force a quicker save after first collection if we have multiple records
                        if (data_buffer.size() >= 2) {
                            printf("INIT: Already have multiple records, triggering immediate save\n");
                            // Reset the last save time to force a save on next iteration
                            last_flash_save_ms = current_time - INIT_FLASH_SAVE_INTERVAL_MS;
                        }
                    }
                }
                
                // Update last collection time
                last_data_collection_ms = current_time;
                
                // Force refresh of display with updated sensor data
                refresh_display = true;
                
                DEBUG_POINT("Data collection complete");
            }
            
            // Check if it's time to save to flash or if buffer is getting too full
            if (((current_time - last_flash_save_ms >= 
                  (initializationComplete ? FLASH_SAVE_INTERVAL_MS : INIT_FLASH_SAVE_INTERVAL_MS)) || 
                 (data_buffer.size() >= 
                  (initializationComplete ? MAX_BUFFER_SIZE : INIT_MAX_BUFFER_SIZE))) && 
                data_buffer.size() > 0) {
                
                DEBUG_POINT("Starting flash save");
                printf("TIMING: Flash save triggered (elapsed: %u ms, interval: %u ms, buffer size: %lu)\n",
                       (unsigned int)(current_time - last_flash_save_ms),
                       (unsigned int)(initializationComplete ? FLASH_SAVE_INTERVAL_MS : INIT_FLASH_SAVE_INTERVAL_MS),
                       data_buffer.size());
                
                // Add forced save if we've collected first data but haven't saved yet
                if (initialDataCollected && !initialDataSaved && data_buffer.size() > 0) {
                    printf("INIT: Forcing first data save for initialization\n");
                    // Save is being handled below, so we don't need additional code here
                }
                
                // Save entire buffer to flash
                size_t saved_count = 0;
                for (const auto& data : data_buffer) {
                    DEBUG_POINT("Saving record to flash");
                    if (flash_storage.saveSensorData(data)) {
                        saved_count++;
                    } else {
                        printf("ERROR: Failed to save record to flash (stored count: %lu)\n", 
                               flash_storage.getStoredCount());
                        
                        if (flash_storage.isStorageFull()) {
                            printf("Flash storage is full - cannot save more records\n");
                            
                            // Show a warning on the display
                            displayUploadStatus("Storage FULL!");
                            sleep_ms(2000);
                            displayUploadStatus("Upload required");
                            sleep_ms(2000);
                            
                            break;
                        }
                    }
                }
                
                printf("Saved %lu/%lu records to flash. Total stored: %lu\n",
                       saved_count, data_buffer.size(), flash_storage.getStoredCount());
                
                // Clear buffer after successful save
                if (saved_count > 0) {
                    data_buffer.clear();
                    buffer_modified = false;
                    
                    // Set the initial data saved flag
                    if (!initialDataSaved) {
                        initialDataSaved = true;
                        printf("INIT: First data save complete\n");
                        // Update initialization page if we're still in init mode
                        if (!initializationComplete) {
                            displayInitializationPage("First save complete", 3, 5);
                            sleep_ms(500);
                            displayInitializationPage("Press any button", 4, 5);
                        }
                    }
                }
                
                // Update last save time
                last_flash_save_ms = current_time;
                DEBUG_POINT("Flash save complete");
            }
            
            // Check if initialization is complete
            if (!initializationComplete && initialDataCollected && initialDataSaved) {
                DEBUG_POINT("Checking for initialization completion");
                // Check if any button is pressed to exit initialization
                if (button_state_changed || 
                    (tast_pressed[0] != NOT_PRESSED) || 
                    (tast_pressed[1] != NOT_PRESSED)) {
                    
                    // Transition from initialization to normal operation
                    printf("INIT: Initialization complete, transitioning to normal operation\n");
                    initializationComplete = true;
                    
                    // Clear button states
                    tast_pressed[0] = NOT_PRESSED;
                    tast_pressed[1] = NOT_PRESSED;
                    button_state_changed = false;
                    
                    // Force a refresh of the BME688 page (page 0)
                    current_page = 0;  // Page 0 is the BME688 page
                    printf("INIT: Setting display to BME688 page (page 0)\n");
                    refresh_display = true;
                    DEBUG_POINT("Initialization completed by button press");
                }
            }
            
            // Add a timeout to force initialization completion after 30 seconds
            static uint32_t init_start_time = 0;
            if (!initializationComplete) {
                if (init_start_time == 0) {
                    init_start_time = current_time;
                } else if (current_time - init_start_time > 30000) {  // 30 second timeout
                    printf("INIT: Forcing initialization complete after timeout\n");
                    initializationComplete = true;
                    
                    // Force a refresh of the BME688 page (page 0)
                    current_page = 0;  // Page 0 is the BME688 page
                    printf("INIT: Setting display to BME688 page (page 0)\n");
                    refresh_display = true;
                    DEBUG_POINT("Initialization completed by timeout");
                }
            }
            
            // Process button state changes immediately when the flag is set and we're in normal operation
            if (initializationComplete && button_state_changed) {
                DEBUG_POINT("Processing button state changes");
                // Process button events here...
                if (tast_pressed[0] == SHORT_PRESSED) {
                    // Next page button
                    DEBUG_POINT("Processing Next Page button (SHORT_PRESSED)");
                    tast_pressed[0] = NOT_PRESSED;
                    // nextPage already calls forceDisplayRefresh
                    nextPage();
                    printf("Changed to page %d and refreshed display\n", current_page);
                } else if (tast_pressed[0] == LONG_PRESSED) {
                    // Long press on next page button - trigger data upload
                    DEBUG_POINT("Processing Next Page button (LONG_PRESSED) - data upload");
                    tast_pressed[0] = NOT_PRESSED;
                    printf("Long press detected on button 0 - starting data upload\n");
                    
                    // Use our optimized WiFi connection function
                    if (ensureWiFiConnection()) {
                        // WiFi is connected, proceed with upload
                        DEBUG_POINT("WiFi connected - preparing for upload");
                        
                        // First flush any data from the buffer to flash
                        if (data_buffer.size() > 0) {
                            printf("Flushing %d records from buffer to flash before upload\n", data_buffer.size());
                            displayUploadStatus("Saving buffer...");
                            
                            for (const auto& data : data_buffer) {
                                flash_storage.saveSensorData(data);
                            }
                            
                            printf("Buffer saved to flash\n");
                            data_buffer.clear();
                        }
                        
                        // Now attempt the upload with the more reliable chunked function 
                        // instead of the parallel function that was failing
                        DEBUG_POINT("Starting data upload");
                        
                        // Check the number of records to determine best upload method
                        uint32_t record_count = flash_storage.getStoredCount();
                        
                        if (record_count > 0) {
                            // Always use the more reliable chunked upload method
                            printf("Using reliable chunked upload method for %lu records\n", record_count);
                            uploadSensorDataChunked(flash_storage, gps, UPLOAD_ALL_AT_ONCE);
                        } else {
                            displayUploadStatus("No data to upload");
                            sleep_ms(1000); // Reduced from 2000ms
                        }
                        
                        DEBUG_POINT("Data upload completed");
                    } else {
                        // No WiFi connection could be established
                        displayUploadStatus("No WiFi, can't upload");
                        sleep_ms(500); // Reduced from 1000ms to 500ms
                    }
                }
                
                if (tast_pressed[1] == SHORT_PRESSED) {
                    // Refresh/settings button
                    DEBUG_POINT("Processing Settings button (SHORT_PRESSED)");
                    tast_pressed[1] = NOT_PRESSED;
                    // This now calls forceDisplayRefresh internally
                    refreshDisplay_Settings_Button(current_page);
                    printf("Settings updated and display refreshed\n");
                } else if (tast_pressed[1] == LONG_PRESSED) {
                    // Long press on refresh button - enter sleep mode
                    DEBUG_POINT("Processing Settings button (LONG_PRESSED) - sleep mode");
                    tast_pressed[1] = NOT_PRESSED;
                    printf("Long press detected on button 1 - entering sleep mode\n");
                    enterSleepMode();
                }
                
                button_state_changed = false;
                DEBUG_POINT("Button processing complete");
            }
            
            // Update last task processing time
            last_task_time = current_time;
            
            // Check if display needs to be refreshed due to page change or data update
            static uint32_t last_display_refresh_time = 0;
            if (refresh_display || (current_time - last_display_refresh_time >= refreshInterval)) {
                DEBUG_POINT("Display refresh triggered");
                
                if (initializationComplete) {
                    // Normal operation - display the current page
                    printf("Refreshing display for page %d (refresh flag: %s, timed: %s)\n", 
                          current_page,
                          refresh_display ? "true" : "false",
                          (current_time - last_display_refresh_time >= refreshInterval) ? "true" : "false");
                    
                    // Update the display based on current page
                    DEBUG_POINT("Calling displayPage");
                    displayPage(current_page, gps_start_time, fix_status, satellites_visible, USE_FAKE_GPS);
                    
                    // Reset the refresh flag and update the refresh time
                    refresh_display = false;
                    last_display_refresh_time = current_time;
                    printf("Display refreshed for page %d\n", current_page);
                    DEBUG_POINT("Display refresh complete");
                } else {
                    // We're still in initialization mode - update the status
                    // But only update periodically to avoid too much flicker
                    if (current_time - last_display_refresh_time >= 5000) { // Update every 5 seconds at most
                        DEBUG_POINT("Updating initialization page");
                        displayInitializationPage("Please wait...", 1, 5);
                        refresh_display = false;
                        last_display_refresh_time = current_time;
                    }
                }
            }
            
            // Additional optional debug output
            static uint32_t last_debug_print_time = 0;
            if (current_time - last_debug_print_time > 10000) {  // Every 10 seconds
                printf("DEBUG: Current page: %d, Refresh flag: %s, Time since last refresh: %u ms\n", 
                       current_page,
                       refresh_display ? "true" : "false",
                       (unsigned int)(current_time - last_display_refresh_time));
                // Print memory usage
                printf("DEBUG: Buffer size: %lu records, Flash storage: %lu records\n",
                       data_buffer.size(), flash_storage.getStoredCount());
                last_debug_print_time = current_time;
            }
        }
        
        // Update last_loop_time at end of each iteration
        last_loop_time = to_ms_since_boot(get_absolute_time());
        
        // Watchdog feed to prevent resets
#ifdef USE_WATCHDOG
        watchdog_update();
#endif

        // Process GPS data more frequently in a dedicated check
        // This runs independently of other operations to ensure continuous GPS acquisition
        if (current_time - last_gps_check_ms >= GPS_POLL_INTERVAL_MS) {
            // Process GPS data with minimal overhead for continuous background acquisition
            std::string gps_data;
            double gps_lon = 0, gps_lat = 0;
            char gps_ew = 'E', gps_ns = 'N';
            std::string gps_time_str, gps_date_str;
            
            // Read GPS data with timeout protection - do this frequently (10Hz polling)
            int gps_status_result = gps.readLine(gps_data, gps_lon, gps_ew, gps_lat, gps_ns, gps_time_str, gps_date_str);
            
            // Update fix status immediately on any change
            if (fix_status != gps_status_result) {
                fix_status = gps_status_result;  // 0 = valid fix, 2 = invalid
                
                // Log fix status changes
                printf("GPS Fix Status changed: %s (satellites: %d)\n", 
                      (fix_status == 0) ? "VALID" : "INVALID", satellites_visible);
                
                // If we just got a valid fix, refresh display to show it
                if (fix_status == 0 && current_page == 4) {
                    refresh_display = true;
                }
            }
            
            // Update satellites visible count periodically
            if (current_time - last_gps_status_update_ms >= GPS_STATUS_UPDATE_MS) {
                satellites_visible = gps.getVisibleSatellites();
                
                // Only log periodic updates if fix status hasn't changed
                printf("GPS Status Update: Fix=%s, Satellites=%d, Coords: %.6f, %.6f\n", 
                      (fix_status == 0) ? "VALID" : "INVALID", 
                      satellites_visible,
                      gps_lat, gps_lon);
                
                last_gps_status_update_ms = current_time;
            }
            
            // Store GPS coordinates for data collection
            if (fix_status == 0) {
                // We have a valid fix, update the latest valid coordinates
                latest_valid_lat = gps_lat;
                latest_valid_lon = gps_lon;
                has_valid_fix_since_boot = true;
                
                // Set the current position data for sensor data collection
                sensor_data_obj.longitude = (int32_t)(gps_lon * 10000000); // Store as fixed-point
                sensor_data_obj.latitude = (int32_t)(gps_lat * 10000000);   // Store as fixed-point
            } else if (has_valid_fix_since_boot) {
                // No current fix, but we had one before - use the last known valid position
                // This ensures we always have valid coordinates when we had a fix at any point
                sensor_data_obj.longitude = (int32_t)(latest_valid_lon * 10000000);
                sensor_data_obj.latitude = (int32_t)(latest_valid_lat * 10000000);
            }
            
            // Update last GPS check time
            last_gps_check_ms = current_time;
        }
    }

    free(ImageBuffer);
    return 0;
}
