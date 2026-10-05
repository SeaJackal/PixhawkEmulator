add_library(controller controller.cpp)
target_include_directories(controller PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(controller PUBLIC cxx_std_17)
target_link_libraries(controller PUBLIC controller_state_machine battery_model telemetry)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
