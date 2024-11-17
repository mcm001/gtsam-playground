cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_STANDARD 20)

project(gtsam-playground)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# redirect output binaries
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

include(FetchContent)

# Grab genresources
include(GenResources)

add_compile_options(-fno-omit-frame-pointer)
set(CXXFLAGS "${CXXFLAGS} -fno-omit-frame-pointer")
set(CMAKE_INSTALL_RPATH "lib")

add_subdirectory(gtsam)
add_subdirectory(allwpilib)

# OpenCV stuff

set(OPENCV_YEAR "frc2023")
set(OPENCV_VERSION "4.6.0-5")
if (NOT DEFINED  OPENCV_ARCH)
  set(OPENCV_ARCH "linuxx86-64")
endif()

# type can be "", "debug", "static", or "staticdebug"
set(OPENCV_TYPE "static")

# Download opencv, and save the path
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
  "${opencv_lib_SOURCE_DIR}/**/*"
)
SET(OPENCV_INCLUDE_PATH ${opencv_header_SOURCE_DIR})

# Generate gtsam-localizer resources
generate_resources(src/resources/ generated/main/cpp GTSAMLOCALIZER meme localizer_resources_src)

add_library(gtsam-localizer
  src/localizer.cpp
  src/TagModel.cpp
  src/gtsam_utils.cpp
  src/config.cpp
  src/camera_listener.cpp
  src/odom_listener.cpp
  src/data_publisher.cpp
  ${localizer_resources_src}
)
target_link_libraries(gtsam-localizer PUBLIC
  gtsam
  gtsam_unstable
  apriltag
  ntcore
  wpimath
)
target_compile_options(gtsam-localizer PRIVATE -Wno-deprecated-enum-enum-conversion)

add_executable(gtsam-node
  src/gtsam_tags_node.cpp
)
target_compile_options(gtsam-node PRIVATE -Wno-deprecated-enum-enum-conversion)
target_link_libraries(gtsam-node gtsam-localizer)

add_library(tag-mapper
  src/sfm_mapper/sfm_mapper.cpp
  src/sfm_mapper/PhotonPoseEstimator.cpp
  src/sfm_mapper/helpers.cpp
  src/sfm_mapper/mapper_nt_iface.cpp
  src/sfm_mapper/wpilog_reader.cpp
)
target_compile_options(tag-mapper PUBLIC -Wno-deprecated-enum-enum-conversion)
target_link_libraries(tag-mapper gtsam-localizer wpilibc ${OPENCV_LIB_PATH})
target_include_directories(tag-mapper PUBLIC src ${OPENCV_INCLUDE_PATH})

add_executable(tag-mapper-live
  src/sfm_mapper/main_live.cpp
)
add_executable(tag-mapper-replay
  src/sfm_mapper/main_replay.cpp
)
target_link_libraries(tag-mapper-live tag-mapper)
target_link_libraries(tag-mapper-replay tag-mapper)

install(TARGETS tag-mapper-live tag-mapper-replay gtsam-node)

# clang-tidy
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=-*,readability-*)

add_executable(constrained_solvepnp_test 
  src/ConstrainedPnp.cpp
  src/TagModel.cpp
  src/gtsam_utils.cpp)
target_link_libraries(constrained_solvepnp_test 
  gtsam 
  gtsam_unstable 
  wpimath
  apriltag
)

set(CMAKE_CXX_CLANG_TIDY "")

install(TARGETS gtsam-node)

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


add_executable(
  tag_mapper_test
  test/Test_SFM.cpp
)
target_link_libraries(
  tag_mapper_test
  GTest::gtest_main tag-mapper
)
target_include_directories(tag_mapper_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(tag_mapper_test PRIVATE -g -Og)
gtest_discover_tests(tag_mapper_test)
