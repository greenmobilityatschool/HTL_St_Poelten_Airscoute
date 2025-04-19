# Define the library and add font sources
add_library(epd_fonts STATIC font8.c font12.c font16.c font20.c font24.c)

# Include directory for this library
target_include_directories(epd_fonts PUBLIC ${CMAKE_CURRENT_LIST_DIR})
