add_library(simulation_time INTERFACE)
target_include_directories(simulation_time INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(simulation_time INTERFACE cxx_std_17)
