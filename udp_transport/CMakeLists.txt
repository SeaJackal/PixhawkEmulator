find_package(Boost 1.70 REQUIRED)

add_library(udp_transport udp_transport.cpp)
target_include_directories(udp_transport PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(udp_transport PUBLIC Boost::headers Threads::Threads)
target_compile_features(udp_transport PUBLIC cxx_std_17)
