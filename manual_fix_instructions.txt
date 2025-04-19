// This is a fixed implementation of the forceDisplayRefresh function
// Put this at the top of your file, after all the includes but before any function definitions

// These are the global variables needed - COPY THESE TO THE TOP OF YOUR FILE
// They MUST be defined before any function that uses them
volatile int current_page = 0;                   // Current display page
absolute_time_t gps_start_time;                  // GPS start time for time display
int fix_status = 2;                              // GPS fix status (0=valid, 2=invalid) 
int satellites_visible = 0;                      // Number of visible satellites

// Copy this simplified forceDisplayRefresh implementation
void forceDisplayRefresh() {
    // Set the refresh flag to true
    refresh_display = true;
    
    // Force immediate refresh instead of waiting for the next cycle
    printf("Forcing immediate display refresh for page %d\n", current_page);
    
    // Call the display function with the needed variables
    displayPage(current_page, gps_start_time, fix_status, satellites_visible, USE_FAKE_GPS);
    
    printf("Display refreshed for page %d\n", current_page);
}

// Copy this implementation of nextPage that uses the simplified forceDisplayRefresh
void nextPage() {
    // Increment the current page index and wrap around if needed
    current_page = (current_page + 1) % PAGE_COUNT;
    
    // Print a message for debugging
    printf("Switched to page %d\n", current_page);
    
    // Force immediate display refresh
    forceDisplayRefresh();
}

// Copy this implementation of refreshDisplay_Settings_Button
void refreshDisplay_Settings_Button(int page) {
    if (current_page == 3) {  // If on the settings page
        // Cycle through refresh intervals...
        // Toggle fast refresh on double-press...
        // Rest of your existing code...
        
        // Force immediate refresh
        forceDisplayRefresh();
    } else {
        // Just refresh the display
        forceDisplayRefresh();
    }
} 