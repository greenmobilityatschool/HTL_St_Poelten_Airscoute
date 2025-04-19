# Define the library and add GUI sources
add_library(epd_gui_paint STATIC GUI_Paint.c)

# Include directory for this library
target_include_directories(epd_gui_paint PUBLIC ${CMAKE_CURRENT_LIST_DIR})
