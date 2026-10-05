add_library(dynamic_model dynamic_model.cpp)
target_link_libraries(dynamic_model PUBLIC simulation_time)
target_include_directories(dynamic_model PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(dynamic_model PUBLIC cxx_std_17)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
