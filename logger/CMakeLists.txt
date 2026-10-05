add_library(logger logger.cpp)
target_link_libraries(logger PUBLIC simulation_time)
target_include_directories(logger PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(logger PUBLIC cxx_std_17)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
