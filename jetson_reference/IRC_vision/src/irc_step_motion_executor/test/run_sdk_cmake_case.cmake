if(NOT DEFINED PACKAGE_SOURCE_DIR OR NOT DEFINED TEST_BINARY_DIR)
  message(FATAL_ERROR "PACKAGE_SOURCE_DIR and TEST_BINARY_DIR are required")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}"
    -S "${PACKAGE_SOURCE_DIR}"
    -B "${TEST_BINARY_DIR}"
    -DBUILD_TESTING=ON
    -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON
    "-DROBOT_MOTION_SDK_DIR=${SDK_DIR}"
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_stdout
  ERROR_VARIABLE configure_stderr)

set(configure_output "${configure_stdout}\n${configure_stderr}")

if(EXPECT_SUCCESS)
  if(NOT configure_result EQUAL 0)
    message(FATAL_ERROR
      "Expected SDK configure success, got ${configure_result}:\n${configure_output}")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${TEST_BINARY_DIR}"
      --target
        irc_step_motion_sdk_compile_probe
        robot_motion_player_backend
        production_robot_motion_runtime_factory
        sdk_hardware_preflight_core
        sdk_hardware_preflight
        sdk_motion_executor
        test_production_robot_motion_runtime_factory
        test_sdk_hardware_preflight_core
        test_motion_backend_factory
    RESULT_VARIABLE build_result
    OUTPUT_VARIABLE build_stdout
    ERROR_VARIABLE build_stderr)
  if(NOT build_result EQUAL 0)
    message(FATAL_ERROR
      "SDK compile probe build failed:\n${build_stdout}\n${build_stderr}")
  endif()
  execute_process(
    COMMAND "${TEST_BINARY_DIR}/test_production_robot_motion_runtime_factory"
    RESULT_VARIABLE runtime_factory_test_result
    OUTPUT_VARIABLE runtime_factory_test_stdout
    ERROR_VARIABLE runtime_factory_test_stderr)
  if(NOT runtime_factory_test_result EQUAL 0)
    message(FATAL_ERROR
      "SDK-enabled production runtime factory test failed:\n"
      "${runtime_factory_test_stdout}\n${runtime_factory_test_stderr}")
  endif()
  execute_process(
    COMMAND "${TEST_BINARY_DIR}/test_motion_backend_factory"
    RESULT_VARIABLE factory_test_result
    OUTPUT_VARIABLE factory_test_stdout
    ERROR_VARIABLE factory_test_stderr)
  if(NOT factory_test_result EQUAL 0)
    message(FATAL_ERROR
      "SDK-enabled factory test failed:\n"
      "${factory_test_stdout}\n${factory_test_stderr}")
  endif()
  execute_process(
    COMMAND "${TEST_BINARY_DIR}/test_sdk_hardware_preflight_core"
    RESULT_VARIABLE preflight_core_test_result
    OUTPUT_VARIABLE preflight_core_test_stdout
    ERROR_VARIABLE preflight_core_test_stderr)
  if(NOT preflight_core_test_result EQUAL 0)
    message(FATAL_ERROR
      "SDK hardware preflight core test failed:\n"
      "${preflight_core_test_stdout}\n${preflight_core_test_stderr}")
  endif()
else()
  if(configure_result EQUAL 0)
    message(FATAL_ERROR "Expected SDK configure failure, but it succeeded")
  endif()
  if(EXPECTED_ERROR AND NOT configure_output MATCHES "${EXPECTED_ERROR}")
    message(FATAL_ERROR
      "Configure failed without expected text '${EXPECTED_ERROR}':\n${configure_output}")
  endif()
endif()
