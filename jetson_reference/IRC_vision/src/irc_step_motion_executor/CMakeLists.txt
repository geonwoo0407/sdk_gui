cmake_minimum_required(VERSION 3.8)
project(irc_step_motion_executor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(IRC_STEP_ENABLE_ROBOT_MOTION_SDK
  "Enable inclusion of an externally supplied RobotMotionPlayer SDK" OFF)
set(ROBOT_MOTION_SDK_DIR "" CACHE PATH
  "External RobotMotionPlayer SDK source directory (empty by default)")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
pkg_check_modules(JSON_C REQUIRED IMPORTED_TARGET json-c)

if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  if(NOT ROBOT_MOTION_SDK_DIR)
    message(FATAL_ERROR
      "IRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON requires ROBOT_MOTION_SDK_DIR")
  endif()
  if(NOT IS_DIRECTORY "${ROBOT_MOTION_SDK_DIR}")
    message(FATAL_ERROR
      "ROBOT_MOTION_SDK_DIR is not an existing directory: ${ROBOT_MOTION_SDK_DIR}")
  endif()
  foreach(required_sdk_file
      CMakeLists.txt
      robot_motion_player.hpp
      robot_motion_player.cpp)
    if(NOT EXISTS "${ROBOT_MOTION_SDK_DIR}/${required_sdk_file}")
      message(FATAL_ERROR
        "ROBOT_MOTION_SDK_DIR is missing required file: ${required_sdk_file}")
    endif()
  endforeach()
  message(STATUS
    "SDK inclusion enabled; no SDK backend is instantiated by this package")
  add_subdirectory("${ROBOT_MOTION_SDK_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motion_sdk" EXCLUDE_FROM_ALL)
  if(NOT TARGET robot_control)
    message(FATAL_ERROR
      "External SDK must create a CMake target named robot_control")
  endif()

  add_library(irc_step_motion_sdk_compile_probe SHARED
    src/sdk_compile_probe.cpp)
  target_compile_features(irc_step_motion_sdk_compile_probe PUBLIC cxx_std_17)
  target_include_directories(irc_step_motion_sdk_compile_probe PRIVATE
    "${ROBOT_MOTION_SDK_DIR}")
  target_link_libraries(irc_step_motion_sdk_compile_probe PRIVATE robot_control)
else()
  message(STATUS
    "Building catalog-only mode (RobotMotionPlayer SDK is not included or linked)")
endif()

add_library(catalog_only_core src/catalog_only_core.cpp)
target_compile_features(catalog_only_core PUBLIC cxx_std_17)
target_include_directories(catalog_only_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(catalog_only_core PkgConfig::JSON_C yaml-cpp)

add_library(sdk_executor_core src/sdk_executor_core.cpp)
target_compile_features(sdk_executor_core PUBLIC cxx_std_17)
target_include_directories(sdk_executor_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(sdk_executor_core PUBLIC catalog_only_core)

if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  add_library(
    robot_motion_player_backend
    src/robot_motion_player_backend.cpp)
  target_compile_features(robot_motion_player_backend PUBLIC cxx_std_20)
  target_include_directories(robot_motion_player_backend PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
  target_link_libraries(
    robot_motion_player_backend PUBLIC sdk_executor_core robot_control)
  install(TARGETS robot_motion_player_backend
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
endif()

add_library(simulated_motion_backend src/simulated_motion_backend.cpp)
target_compile_features(simulated_motion_backend PUBLIC cxx_std_17)
target_include_directories(simulated_motion_backend PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(simulated_motion_backend PUBLIC sdk_executor_core)

add_library(
  robot_motion_runtime_config
  src/robot_motion_runtime_config.cpp)
target_compile_features(robot_motion_runtime_config PUBLIC cxx_std_17)
target_include_directories(robot_motion_runtime_config PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  add_library(
    production_robot_motion_runtime_factory
    src/production_robot_motion_runtime_factory.cpp)
  target_compile_features(
    production_robot_motion_runtime_factory PUBLIC cxx_std_20)
  target_include_directories(production_robot_motion_runtime_factory PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
  target_link_libraries(production_robot_motion_runtime_factory PUBLIC
    robot_motion_runtime_config robot_motion_player_backend robot_control)
  install(TARGETS production_robot_motion_runtime_factory
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)

  add_library(
    sdk_hardware_preflight_core
    src/sdk_hardware_preflight_core.cpp)
  target_compile_features(sdk_hardware_preflight_core PUBLIC cxx_std_17)
  target_include_directories(sdk_hardware_preflight_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
  target_link_libraries(sdk_hardware_preflight_core PUBLIC
    production_robot_motion_runtime_factory)

  add_executable(
    sdk_hardware_preflight
    src/sdk_hardware_preflight.cpp)
  target_compile_features(sdk_hardware_preflight PUBLIC cxx_std_17)
  target_link_libraries(
    sdk_hardware_preflight PRIVATE sdk_hardware_preflight_core)

  install(TARGETS sdk_hardware_preflight_core sdk_hardware_preflight
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

add_library(motion_backend_factory src/motion_backend_factory.cpp)
target_compile_features(motion_backend_factory PUBLIC cxx_std_17)
target_include_directories(motion_backend_factory PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(
  motion_backend_factory
  PUBLIC simulated_motion_backend robot_motion_runtime_config)
if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  target_compile_definitions(
    motion_backend_factory
    PRIVATE IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT=1)
else()
  target_compile_definitions(
    motion_backend_factory
    PRIVATE IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT=0)
endif()

add_library(sdk_executor_driver src/sdk_executor_driver.cpp)
target_compile_features(sdk_executor_driver PUBLIC cxx_std_17)
target_include_directories(sdk_executor_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_sources(sdk_executor_driver PRIVATE
  src/startup_pose_gate.cpp src/startup_pose_catalog.cpp)
target_link_libraries(sdk_executor_driver PUBLIC
  sdk_executor_core robot_motion_runtime_config PkgConfig::JSON_C)

add_executable(catalog_only_node src/catalog_only_node.cpp)
target_compile_features(catalog_only_node PUBLIC cxx_std_17)
target_link_libraries(catalog_only_node catalog_only_core)
ament_target_dependencies(catalog_only_node ament_index_cpp rclcpp std_msgs)

add_executable(sdk_motion_executor src/sdk_executor_node.cpp)
target_compile_features(sdk_motion_executor PUBLIC cxx_std_17)
target_link_libraries(
  sdk_motion_executor sdk_executor_driver motion_backend_factory)
if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  target_compile_definitions(
    sdk_motion_executor
    PRIVATE IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT=1)
  target_link_libraries(
    sdk_motion_executor production_robot_motion_runtime_factory)
else()
  target_compile_definitions(
    sdk_motion_executor
    PRIVATE IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT=0)
endif()
ament_target_dependencies(
  sdk_motion_executor ament_index_cpp rclcpp std_msgs)

install(TARGETS
    catalog_only_core
    sdk_executor_core
    simulated_motion_backend
    robot_motion_runtime_config
    motion_backend_factory
    sdk_executor_driver
    catalog_only_node
    sdk_motion_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../artifacts/robot_motions_runtime.json"
  DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/manual_motion_request.py
  scripts/validate_motion_catalog.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_catalog_only_core test/test_catalog_only_core.cpp)
  if(TARGET test_catalog_only_core)
    target_link_libraries(test_catalog_only_core catalog_only_core)
    target_compile_definitions(test_catalog_only_core PRIVATE
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml"
      TEST_CONTRACT_VECTORS="${CMAKE_CURRENT_SOURCE_DIR}/test/legacy_catalog_contract_vectors.json")
  endif()
  ament_add_gtest(
    test_motion_json_catalog test/test_motion_json_catalog.cpp)
  if(TARGET test_motion_json_catalog)
    target_link_libraries(test_motion_json_catalog PkgConfig::JSON_C)
    target_compile_definitions(test_motion_json_catalog PRIVATE
      TEST_MOTION_CANDIDATE_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../../artifacts/robot_motions_candidates.json")
  endif()
  ament_add_gtest(test_sdk_executor_core test/test_sdk_executor_core.cpp)
  if(TARGET test_sdk_executor_core)
    target_include_directories(test_sdk_executor_core PRIVATE test)
    target_link_libraries(test_sdk_executor_core sdk_executor_core)
    target_compile_definitions(test_sdk_executor_core PRIVATE
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
  endif()
  if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
    ament_add_gtest(
      test_robot_motion_player_backend
      test/test_robot_motion_player_backend.cpp)
    if(TARGET test_robot_motion_player_backend)
      target_link_libraries(
        test_robot_motion_player_backend robot_motion_player_backend)
    endif()
  endif()
  ament_add_gtest(
    test_simulated_motion_backend
    test/test_simulated_motion_backend.cpp)
  if(TARGET test_simulated_motion_backend)
    target_link_libraries(
      test_simulated_motion_backend simulated_motion_backend)
  endif()
  ament_add_gtest(
    test_robot_motion_runtime_config
    test/test_robot_motion_runtime_config.cpp)
  if(TARGET test_robot_motion_runtime_config)
    target_link_libraries(
      test_robot_motion_runtime_config robot_motion_runtime_config)
    target_compile_definitions(test_robot_motion_runtime_config PRIVATE
      TEST_EXISTING_RUNTIME_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
  endif()
  ament_add_gtest(
    test_robot_motion_parameter_types
    test/test_robot_motion_parameter_types.cpp)
  if(TARGET test_robot_motion_parameter_types)
    ament_target_dependencies(test_robot_motion_parameter_types rclcpp)
  endif()
  if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK AND
    EXISTS "${ROBOT_MOTION_SDK_DIR}/fake_robot_motion_sdk_test_support.hpp")
    ament_add_gtest(
      test_production_robot_motion_runtime_factory
      test/test_production_robot_motion_runtime_factory.cpp)
    if(TARGET test_production_robot_motion_runtime_factory)
      target_link_libraries(test_production_robot_motion_runtime_factory
        production_robot_motion_runtime_factory)
      target_compile_definitions(
        test_production_robot_motion_runtime_factory PRIVATE
        TEST_EXISTING_RUNTIME_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
    endif()
    ament_add_gtest(
      test_sdk_hardware_preflight_core
      test/test_sdk_hardware_preflight_core.cpp)
    if(TARGET test_sdk_hardware_preflight_core)
      target_link_libraries(
        test_sdk_hardware_preflight_core sdk_hardware_preflight_core)
    endif()
  endif()
  ament_add_gtest(
    test_motion_backend_factory
    test/test_motion_backend_factory.cpp)
  if(TARGET test_motion_backend_factory)
    target_link_libraries(
      test_motion_backend_factory motion_backend_factory)
    target_compile_definitions(test_motion_backend_factory PRIVATE
      TEST_EXISTING_RUNTIME_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
    if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
      target_compile_definitions(
        test_motion_backend_factory
        PRIVATE EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT=1)
    else()
      target_compile_definitions(
        test_motion_backend_factory
        PRIVATE EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT=0)
    endif()
  endif()
  ament_add_gtest(
    test_sdk_executor_driver test/test_sdk_executor_driver.cpp)
  if(TARGET test_sdk_executor_driver)
    target_include_directories(test_sdk_executor_driver PRIVATE test)
    target_link_libraries(test_sdk_executor_driver sdk_executor_driver)
    target_compile_definitions(test_sdk_executor_driver PRIVATE
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
  endif()
  ament_add_gtest(test_startup_pose_gate test/test_startup_pose_gate.cpp)
  if(TARGET test_startup_pose_gate)
    target_include_directories(test_startup_pose_gate PRIVATE test)
    target_link_libraries(test_startup_pose_gate sdk_executor_driver)
    target_compile_definitions(test_startup_pose_gate PRIVATE
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
  endif()
  ament_add_gtest(
    test_startup_pose_catalog test/test_startup_pose_catalog.cpp)
  if(TARGET test_startup_pose_catalog)
    target_link_libraries(test_startup_pose_catalog sdk_executor_driver)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  add_launch_test(
    test/test_sdk_executor_node_launch.py
    TIMEOUT 30
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs"
    APPEND_ENV "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
    APPEND_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
  if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
    set(expect_robot_motion_player_backend_built 1)
  else()
    set(expect_robot_motion_player_backend_built 0)
  endif()
  add_launch_test(
    test/test_backend_selection_rejected_launch.py
    TIMEOUT 30
    ENV
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs"
      "EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT=${expect_robot_motion_player_backend_built}"
    APPEND_ENV "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
    APPEND_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")

  function(add_sdk_cmake_guard_test test_name sdk_dir expect_success expected_error)
    add_test(
      NAME ${test_name}
      COMMAND ${CMAKE_COMMAND}
        "-DPACKAGE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/sdk_cmake_tests/${test_name}"
        "-DSDK_DIR=${sdk_dir}"
        "-DEXPECT_SUCCESS=${expect_success}"
        "-DEXPECTED_ERROR=${expected_error}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/test/run_sdk_cmake_case.cmake")
  endfunction()

  add_sdk_cmake_guard_test(
    sdk_cmake_empty_path "" FALSE
    "requires ROBOT_MOTION_SDK_DIR")
  add_sdk_cmake_guard_test(
    sdk_cmake_nonexistent_path
    "${CMAKE_CURRENT_BINARY_DIR}/sdk_path_that_does_not_exist"
    FALSE "is not an existing directory")
  add_sdk_cmake_guard_test(
    sdk_cmake_missing_required_files
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/missing_files_sdk"
    FALSE "missing required file: robot_motion_player.hpp")
  add_sdk_cmake_guard_test(
    sdk_cmake_missing_robot_control_target
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/missing_target_sdk"
    FALSE "must create a CMake target named robot_control")
  add_sdk_cmake_guard_test(
    sdk_cmake_fake_sdk_build
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fake_robot_motion_sdk"
    TRUE "")
  add_test(
    NAME sdk_cmake_disabled_build_has_no_hardware_symbols
    COMMAND ${CMAKE_COMMAND}
      "-DPACKAGE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
      "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/sdk_cmake_tests/sdk_disabled"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/run_sdk_disabled_case.cmake")
  add_test(
    NAME test_validate_motion_catalog
    COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_validate_motion_catalog.py")
endif()

ament_export_include_directories(include)
ament_export_libraries(
  catalog_only_core
  sdk_executor_core
  simulated_motion_backend
  robot_motion_runtime_config
  motion_backend_factory
  sdk_executor_driver)
if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  ament_export_libraries(
    robot_motion_player_backend production_robot_motion_runtime_factory
    sdk_hardware_preflight_core)
endif()
ament_package()
