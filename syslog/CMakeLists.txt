add_library(syslog_sender syslog.cpp)
target_include_directories(syslog_sender PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(syslog_sender PUBLIC logger udp_transport)

if(BUILD_TESTING)
    add_executable(syslog_tests tests/syslog_test.cpp)
    target_link_libraries(syslog_tests PRIVATE syslog_sender GTest::gtest_main)
    gtest_discover_tests(syslog_tests)
endif()
