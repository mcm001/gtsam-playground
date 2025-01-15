cmake_minimum_required(VERSION 3.16)

set(CXX_STANDARD 20)

project(gtsam-playground)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH "lib")

add_compile_options(-fno-omit-frame-pointer)

# redirect output binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Versions to grab from Maven
set(FRC_YEAR "frc2025")
set(GTSAM_VERSION "4.3-2")
set(WPILIB_VERSION "4.3-2")

# type can be "", "debug", "static", or "staticdebug"
set(BUILD_TYPE "static")
set(BUILD_ARCH "linuxx86-64") # also valid: windowsx86-64

# Download gtsam
# message("Getting gtsam from https://frcmaven.wpi.edu/artifactory/development/edu/wpi/first/thirdparty/${FRC_YEAR}/gtsam/gtsam-cpp/${GTSAM_VERSION}/gtsam-cpp-${GTSAM_VERSION}-${BUILD_ARCH}${BUILD_TYPE}.zip and https://frcmaven.wpi.edu/artifactory/development/edu/wpi/first/thirdparty/${FRC_YEAR}/gtsam/gtsam-cpp/${GTSAM_VERSION}/gtsam-cpp-${GTSAM_VERSION}-headers.zip")
include(FetchContent)
FetchContent_Declare(
    gtsam_lib
    URL https://frcmaven.wpi.edu/artifactory/development/edu/wpi/first/thirdparty/${FRC_YEAR}/gtsam/gtsam-cpp/${GTSAM_VERSION}/gtsam-cpp-${GTSAM_VERSION}-${BUILD_ARCH}${BUILD_TYPE}.zip
)
FetchContent_MakeAvailable(gtsam_lib)
FetchContent_Declare(
    gtsam_header
    URL https://frcmaven.wpi.edu/artifactory/development/edu/wpi/first/thirdparty/${FRC_YEAR}/gtsam/gtsam-cpp/${GTSAM_VERSION}/gtsam-cpp-${GTSAM_VERSION}-headers.zip
)
FetchContent_MakeAvailable(gtsam_header)
file(GLOB_RECURSE GTSAM_LIBS
  "${gtsam_lib_SOURCE_DIR}/**/*.lib"
  "${gtsam_lib_SOURCE_DIR}/**/*.so*"
  "${gtsam_lib_SOURCE_DIR}/**/*.dylib*"
)
SET(GTSAM_INCLUDE_PATH ${gtsam_header_SOURCE_DIR})

add_library(gtsam-localizer
  src/localizer.cpp
  src/TagModel.cpp
  src/gtsam_utils.cpp
  src/config.cpp
  src/camera_listener.cpp
  src/odom_listener.cpp
  src/data_publisher.cpp
  src/config_listener.cpp
  ${localizer_resources_src}
)

target_include_directories(gtsam-localizer PRIVATE ${GTSAM_INCLUDE_PATH})
target_link_libraries(gtsam-localizer PUBLIC
  ${GTSAM_LIBS}
  apriltag
  ntcore
  wpimath
)
target_compile_options(gtsam-localizer PRIVATE -Wno-deprecated-enum-enum-conversion)

add_executable(gtsam-node
  src/gtsam_tags_node.cpp
)
target_compile_options(gtsam-localizer PRIVATE -Wno-deprecated-enum-enum-conversion)
target_link_libraries(gtsam-node gtsam-localizer)

install(TARGETS gtsam-node)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  localizer_test
  test/Test_Localizer.cpp
  test/Test_Config.cpp
)
target_link_libraries(
  localizer_test
  GTest::gtest_main gtsam-localizer
)
target_include_directories(localizer_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(localizer_test PRIVATE -g -Og)

include(GoogleTest)
gtest_discover_tests(localizer_test)
