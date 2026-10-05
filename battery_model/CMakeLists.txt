add_library(battery_model battery_model.cpp)
target_link_libraries(battery_model PUBLIC simulation_time)
target_include_directories(battery_model PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(battery_model PUBLIC cxx_std_17)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
