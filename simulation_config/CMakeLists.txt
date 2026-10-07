add_library(simulation_config simulation_config.cpp)
target_include_directories(simulation_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(simulation_config PUBLIC cxx_std_17)
target_link_libraries(simulation_config PUBLIC network_emulator udp_transport PRIVATE Boost::json)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
