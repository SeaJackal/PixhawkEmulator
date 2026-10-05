add_library(network_message telemetry_serialization.cpp)
target_include_directories(network_message PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(network_message PUBLIC cxx_std_17)
target_link_libraries(network_message PUBLIC simulation_time telemetry)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
