add_library(controller_state_machine controller_state_machine.cpp)
target_include_directories(controller_state_machine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(controller_state_machine PUBLIC cxx_std_17)
target_link_libraries(controller_state_machine PUBLIC dynamic_model logger)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
