# Download and unpack gtsam at configure time
configure_file(CMakeLists.txt.in gtsam-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtsam-download)

if(result)
  message(FATAL_ERROR "CMake step for gtsam failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtsam-download)

if(result)
  message(FATAL_ERROR "Build step for gtsam failed: ${result}")
endif()

set(GTSAM_BUILD_UNSTABLE ON)
set(GTSAM_ENABLE_TIMING OFF)
set(GTSAM_ALLOW_DEPRECATED_SINCE_V42 OFF)

# Add gtsam directly to our build.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/gtsam-src
                 ${CMAKE_CURRENT_BINARY_DIR}/gtsam-build
                 EXCLUDE_FROM_ALL)
