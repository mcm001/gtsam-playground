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

include(FetchContent)

# Download stuff
function(FETCH_MAVEN repository package libname libversion)
  message("Repo ${repository} package ${package} libname ${libname} version ${libversion}")  

  MESSAGE("Downloading https://frcmaven.wpi.edu/artifactory/${repository}/edu/wpi/first/${package}/${FRC_YEAR}/${libname}/${libname}-cpp/${libversion}/${libname}-cpp-${libversion}-${BUILD_ARCH}${BUILD_TYPE}.zip")
  FetchContent_Declare(
      "${libname}_lib"
      URL https://frcmaven.wpi.edu/artifactory/${repository}/edu/wpi/first/${package}/${libname}-cpp/${libversion}/${libname}-cpp-${libversion}-${BUILD_ARCH}${BUILD_TYPE}.zip
  )
  FetchContent_MakeAvailable("${libname}_lib")
  message("SEARCHING ${${libname}_lib_SOURCE_DIR}")
  file(GLOB_RECURSE ${libname}_LIBS
    "${${libname}_lib_SOURCE_DIR}/**/*.lib"
    "${${libname}_lib_SOURCE_DIR}/**/*.so*"
    "${${libname}_lib_SOURCE_DIR}/**/*.dylib*"
    "${${libname}_lib_SOURCE_DIR}/**/*.a"
  )

  MESSAGE("Downloading https://frcmaven.wpi.edu/artifactory/${repository}/edu/wpi/first/${package}/${FRC_YEAR}/${libname}/${libname}-cpp/${libversion}/${libname}-cpp-${libversion}-headers.zip")
  FetchContent_Declare(
      "${libname}_header"
      URL https://frcmaven.wpi.edu/artifactory/${repository}/edu/wpi/first/${package}/${libname}-cpp/${libversion}/${libname}-cpp-${libversion}-headers.zip
  )
  FetchContent_MakeAvailable("${libname}_header")
  SET("${libname}_INCLUDE_PATH" "${libname}_header_SOURCE_DIR" PARENT_SCOPE)

  message("${libname}_LIBS set to ${${libname}_LIBS}")
  message("${libname}_INCLUDE_PATH set to ${${libname}_header_SOURCE_DIR}")
endfunction()

FETCH_MAVEN(development thirdparty/${FRC_YEAR}/gtsam gtsam ${GTSAM_VERSION})
FETCH_MAVEN(release wpiutil wpiutil 2025.2.1)
FETCH_MAVEN(release wpimath wpimath 2025.2.1)

message("gtsam at ${gtsam_INCLUDE_PATH}")
message("wpimath at ${wpimath_INCLUDE_PATH}")
message("wpiutil at ${wpiutil_INCLUDE_PATH}")

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

target_include_directories(gtsam-localizer SYSTEM PRIVATE
  ${gtsam_INCLUDE_PATH}
  ${wpiutil_INCLUDE_PATH}
  ${wpimath_INCLUDE_PATH}
)
target_link_libraries(gtsam-localizer PUBLIC
  ${gtsam_LIBS}
  ${wpiutil_LIBS}
  ${wpimath_LIBS}
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
