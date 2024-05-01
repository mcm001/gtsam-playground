cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_STANDARD 20)

project(gtsam-playground)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# redirect output binaries
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

# Grab genresources
include(GenResources)

add_compile_options(-fno-omit-frame-pointer)
set(CXXFLAGS "${CXXFLAGS} -fno-omit-frame-pointer")
set(CMAKE_INSTALL_RPATH "lib")

add_subdirectory(gtsam)
add_subdirectory(allwpilib)

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
