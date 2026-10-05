add_library(scheduled_link scheduled_link.cpp)
target_include_directories(scheduled_link PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(scheduled_link PUBLIC network_emulator)
