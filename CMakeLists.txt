cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_STANDARD 20)

project(gtsam-playground)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# Grab genresources
include(GenResources)

add_compile_options(-fno-omit-frame-pointer)
set(CXXFLAGS "${CXXFLAGS} -fno-omit-frame-pointer")
set(CMAKE_INSTALL_RPATH "lib")

set(GTSAM_BUILD_UNSTABLE ON)
# unset(GTSAM_ALLOW_DEPRECATED_SINCE_V42)
set(GTSAM_ENABLE_TIMING OFF)
set(GTSAM_ALLOW_DEPRECATED_SINCE_V42 OFF)
# set (CMAKE_BUILD_TYPE_CACHE ${CMAKE_BUILD_TYPE})
# set (CMAKE_BUILD_TYPE Release)
add_subdirectory(gtsam)
add_subdirectory(allwpilib)
# set (CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_CACHE})
# message(INFO "Build type for our library is: ${CMAKE_BUILD_TYPE}")

# ========= begin opencv stuff ==========

set(OPENCV_YEAR "frc2024")
set(OPENCV_VERSION "4.8.0-2")
# type can be "", "debug", "static", or "staticdebug"
set(OPENCV_TYPE "" CACHE STRING "One of <blank>, debug, static, staticdebug")
set(OPENCV_ARCH "linuxx86-64" CACHE STRING "Opencv Arch, one of linuxarm32, linuxarm64, linuxathena, linuxx86-64, osxarm64, osxx86-64, windowsarm64, windowsx86-64, windowsx86")

# Download opencv, and save the path
include(FetchContent)
FetchContent_Declare(
    opencv_lib
    URL https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/${OPENCV_YEAR}/opencv/opencv-cpp/${OPENCV_VERSION}/opencv-cpp-${OPENCV_VERSION}-${OPENCV_ARCH}${OPENCV_TYPE}.zip
)
FetchContent_MakeAvailable(opencv_lib)

# download OpenCV headers
FetchContent_Declare(
    opencv_header
    URL https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/${OPENCV_YEAR}/opencv/opencv-cpp/${OPENCV_VERSION}/opencv-cpp-${OPENCV_VERSION}-headers.zip
)
FetchContent_MakeAvailable(opencv_header)

# This probably doesn't work great with shared libraries, but I don't care about those right now
file(GLOB_RECURSE OPENCV_LIB_PATH
  "${opencv_lib_SOURCE_DIR}/**/*.lib"
  "${opencv_lib_SOURCE_DIR}/**/*.so*"
)
SET(OPENCV_INCLUDE_PATH ${opencv_header_SOURCE_DIR})
message("Depending on opencv ${OPENCV_LIB_PATH}")

# =========  end opencv stuff  ==========

# Generate gtsam-localizer resources
generate_resources(src/resources/ generated/main/cpp GTSAMLOCALIZER meme localizer_resources_src)

add_library(gtsam-localizer 
  src/localizer.cpp 
  src/TagModel.cpp
  src/gtsam_utils.cpp
  src/config.cpp
  src/camera_listener.cpp
  ${localizer_resources_src}
)
target_link_libraries(gtsam-localizer PUBLIC
  gtsam
  gtsam_unstable
  apriltag
  ntcore
  wpimath
  ${OPENCV_LIB_PATH}
)
target_compile_options(gtsam-localizer PRIVATE -Wno-deprecated-enum-enum-conversion)
target_include_directories(gtsam-localizer PRIVATE ${OPENCV_INCLUDE_PATH})

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

include(GoogleTest)
gtest_discover_tests(localizer_test)


# add_executable(cam_test
#   src/cam_proj_test.cpp
#   src/TagModel.cpp
# )
# target_link_libraries(cam_test
#   gtsam
# )
# target_compile_options(cam_test PRIVATE -Wno-deprecated-enum-enum-conversion)
