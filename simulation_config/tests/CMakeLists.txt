add_executable(simulation_config_tests simulation_config_test.cpp)
target_link_libraries(simulation_config_tests PRIVATE simulation_config GTest::gtest_main)
target_compile_definitions(simulation_config_tests PRIVATE
    SIMULATION_CONFIG_EXAMPLE="${CMAKE_CURRENT_SOURCE_DIR}/../example.json")
gtest_discover_tests(simulation_config_tests)
