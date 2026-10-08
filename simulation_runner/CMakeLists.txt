add_library(simulation_runner INTERFACE)
target_include_directories(simulation_runner INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(simulation_runner INTERFACE controller network_message network_emulator logger Threads::Threads)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
