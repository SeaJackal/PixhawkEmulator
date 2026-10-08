add_library(controller INTERFACE)
target_include_directories(controller INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(controller INTERFACE cxx_std_17)
target_link_libraries(controller INTERFACE controller_state_machine battery_model telemetry)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
