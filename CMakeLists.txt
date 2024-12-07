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
target_compile_options(gtsam-localizer PRIVATE -Wno-deprecated-enum-enum-conversion)
target_link_libraries(gtsam-node gtsam-localizer)

add_library(wpical
  src/wpical/wpical.cpp
  src/wpical/pose_converters.cpp
  src/wpical/gtsam_tag_map.cpp
)
target_link_libraries(wpical PRIVATE gtsam apriltag ${OPENCV_LIB_PATH})
target_include_directories(wpical PRIVATE src ${OPENCV_INCLUDE_PATH})

if (CMAKE_COMPILER_IS_GNUCC)
    target_compile_options(wpical PRIVATE -Wno-deprecated-enum-enum-conversion)
endif()

add_executable(tag-mapper 
  src/tag_mapper/main.cpp
  src/tag_mapper/mapper_nt_iface.cpp
)
target_compile_options(tag-mapper PRIVATE -Wno-deprecated-enum-enum-conversion)
target_link_libraries(tag-mapper gtsam wpilibc apriltag wpical ${OPENCV_LIB_PATH})
target_include_directories(tag-mapper PRIVATE src ${OPENCV_INCLUDE_PATH})

install(TARGETS wpical)
install(TARGETS tag-mapper)
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
  test/Test_helpers.cpp
)
target_link_libraries(
  localizer_test
  GTest::gtest_main gtsam-localizer tag-mapper-lib
)
target_include_directories(localizer_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(localizer_test PRIVATE -g -Og)

include(GoogleTest)
gtest_discover_tests(localizer_test)
