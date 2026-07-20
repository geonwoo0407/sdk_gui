cmake_minimum_required(VERSION 3.16)
project(irc_humanoid_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# 설치 위치 예: /usr/local/include/dynamixel_sdk/dynamixel_sdk.h
find_path(DYNAMIXEL_SDK_INCLUDE_DIR dynamixel_sdk/dynamixel_sdk.h)
find_library(DYNAMIXEL_SDK_LIBRARY NAMES dxl_x64_cpp dynamixel_sdk)

add_library(motion_core
    motion_pattern.cpp
    motion_callback.cpp
)
target_include_directories(motion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(motion_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(test_motion_pattern test_motion_pattern.cpp)
target_link_libraries(test_motion_pattern PRIVATE motion_core)
target_compile_definitions(test_motion_pattern PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_executable(test_motion_callback test_motion_callback.cpp)
target_link_libraries(test_motion_callback PRIVATE motion_core)

enable_testing()
add_test(NAME motion_pattern_test COMMAND test_motion_pattern)
add_test(NAME motion_callback_test COMMAND test_motion_callback)

if(DYNAMIXEL_SDK_INCLUDE_DIR AND DYNAMIXEL_SDK_LIBRARY)
    add_executable(irc_robot
        main.cpp
        step_dynamixel.cpp
        dynamixel_controller.cpp
    )
    target_include_directories(irc_robot PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${DYNAMIXEL_SDK_INCLUDE_DIR}
    )
    target_link_libraries(irc_robot PRIVATE
        motion_core
        Eigen3::Eigen
        Threads::Threads
        ${DYNAMIXEL_SDK_LIBRARY}
    )
    target_compile_options(irc_robot PRIVATE -Wall -Wextra -Wpedantic)
else()
    message(WARNING "DYNAMIXEL SDK not found: irc_robot target will not be built")
endif()
