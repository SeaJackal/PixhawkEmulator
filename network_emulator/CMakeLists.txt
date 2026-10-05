add_library(network_emulator network_emulator.cpp)
target_include_directories(network_emulator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(network_emulator PUBLIC cxx_std_17)
target_link_libraries(network_emulator PUBLIC network_message telemetry)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
