cmake_minimum_required(VERSION 3.20)
project(PixhawkSimulator LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

include(FetchContent)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()
set(BOOST_INCLUDE_LIBRARIES asio json)
FetchContent_Declare(Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.85.0/boost-1.85.0-cmake.tar.xz
    URL_HASH SHA256=0a9cc56ceae46986f5f4d43fe0311d90cf6d2fa9028258a95cab49ffdacf92ad
)
FetchContent_MakeAvailable(Boost)

include(CTest)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)
endif()

add_subdirectory(simulation_time)
add_subdirectory(battery_model)
add_subdirectory(dynamic_model)
add_subdirectory(logger)
add_subdirectory(controller_state_machine)
add_subdirectory(telemetry)
add_subdirectory(network_message)
add_subdirectory(network_emulator)
add_subdirectory(controller)
add_subdirectory(udp_transport)
add_subdirectory(syslog)
add_subdirectory(simulation_runner)
add_subdirectory(simulation_config)

add_executable(pixhawk_simulator main.cpp)
target_link_libraries(pixhawk_simulator PRIVATE simulation_runner udp_transport simulation_config syslog_sender)
