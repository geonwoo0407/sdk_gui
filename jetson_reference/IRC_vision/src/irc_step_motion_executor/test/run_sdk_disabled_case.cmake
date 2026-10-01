if(NOT DEFINED PACKAGE_SOURCE_DIR OR NOT DEFINED TEST_BINARY_DIR)
  message(FATAL_ERROR "PACKAGE_SOURCE_DIR and TEST_BINARY_DIR are required")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}"
    -S "${PACKAGE_SOURCE_DIR}"
    -B "${TEST_BINARY_DIR}"
    -DBUILD_TESTING=OFF
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_stdout
  ERROR_VARIABLE configure_stderr)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR
    "Default SDK-disabled configure failed:\n${configure_stdout}\n${configure_stderr}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${TEST_BINARY_DIR}"
    --target
      catalog_only_core
      sdk_executor_core
      simulated_motion_backend
      robot_motion_runtime_config
      motion_backend_factory
      sdk_executor_driver
      catalog_only_node
      sdk_motion_executor
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_stdout
  ERROR_VARIABLE build_stderr)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "Default SDK-disabled build failed:\n${build_stdout}\n${build_stderr}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${TEST_BINARY_DIR}"
    --target help
  RESULT_VARIABLE target_help_result
  OUTPUT_VARIABLE available_targets
  ERROR_VARIABLE target_help_stderr)
if(NOT target_help_result EQUAL 0)
  message(FATAL_ERROR
    "Could not inspect SDK-disabled targets: ${target_help_stderr}")
endif()
if(available_targets MATCHES "sdk_hardware_preflight")
  message(FATAL_ERROR
    "sdk_hardware_preflight target must not exist when the SDK is disabled")
endif()

find_program(NM_EXECUTABLE nm REQUIRED)
foreach(binary
    "${TEST_BINARY_DIR}/libcatalog_only_core.a"
    "${TEST_BINARY_DIR}/libsdk_executor_core.a"
    "${TEST_BINARY_DIR}/libsimulated_motion_backend.a"
    "${TEST_BINARY_DIR}/librobot_motion_runtime_config.a"
    "${TEST_BINARY_DIR}/libmotion_backend_factory.a"
    "${TEST_BINARY_DIR}/libsdk_executor_driver.a"
    "${TEST_BINARY_DIR}/catalog_only_node"
    "${TEST_BINARY_DIR}/sdk_motion_executor")
  execute_process(
    COMMAND "${NM_EXECUTABLE}" -C "${binary}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE symbols
    ERROR_VARIABLE nm_stderr)
  if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "Could not inspect ${binary}: ${nm_stderr}")
  endif()
  if(symbols MATCHES "RobotMotionPlayer|Dynamixel|robot_control")
    message(FATAL_ERROR
      "SDK-disabled target contains a hardware SDK symbol:\n${symbols}")
  endif()
endforeach()
