cmake_minimum_required(VERSION 3.8)

add_library(robot_control STATIC robot_motion_player.cpp)
target_compile_features(robot_control PUBLIC cxx_std_20)
set_target_properties(robot_control PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(robot_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
