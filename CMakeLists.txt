cmake_minimum_required(VERSION 3.25..3.32)

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
function(FETCH_MAVEN repository package libname libversion libpath includepath)
  message("Repo ${repository} package ${package} libname ${libname} version ${libversion}")  

  FetchContent_Declare(
      "${libname}_lib"
      URL https://frcmaven.wpi.edu/artifactory/${repository}/edu/wpi/first/${package}/${libname}-cpp/${libversion}/${libname}-cpp-${libversion}-${BUILD_ARCH}${BUILD_TYPE}.zip
  )
  FetchContent_MakeAvailable("${libname}_lib")


  FetchContent_Declare(
      "${libname}_header"
      URL https://frcmaven.wpi.edu/artifactory/${repository}/edu/wpi/first/${package}/${libname}-cpp/${libversion}/${libname}-cpp-${libversion}-headers.zip
  )
  FetchContent_MakeAvailable("${libname}_header")

  file(GLOB_RECURSE lib_glob
    "${${libname}_lib_SOURCE_DIR}/**/*.lib"
    "${${libname}_lib_SOURCE_DIR}/**/*.so*"
    "${${libname}_lib_SOURCE_DIR}/**/*.dylib*"
    "${${libname}_lib_SOURCE_DIR}/**/*.a"
  )

  set(${includepath} ${${libname}_header_SOURCE_DIR})
  set(${libpath} ${lib_glob})

  message("Outputting to ${libpath}: ${lib_glob}")
  message("and ${includepath}: ${${libname}_header_SOURCE_DIR}")

  return(PROPAGATE ${includepath} ${libpath})
endfunction()


# Fetch Eigen
set(EIGEN_BUILD_BLAS OFF)
set(EIGEN_BUILD_LAPACK OFF)
set(EIGEN_BUILD_CMAKE_PACKAGE ON)
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY    https://gitlab.com/libeigen/eigen.git
    GIT_TAG           0fb2ed140d4fc0108553ecfb25f2d7fc1a9319a1
    SYSTEM
)
fetchcontent_makeavailable(Eigen3)


# and wpilib libraries
FETCH_MAVEN(
  development 
  thirdparty/${FRC_YEAR}/gtsam 
  gtsam 
  ${GTSAM_VERSION} 
  gtsam_libs 
  gtsam_include_path
)
FETCH_MAVEN(release wpiutil wpiutil 2025.2.1
  wpiutil_libs 
  wpiutil_include_path
)
FETCH_MAVEN(release wpimath wpimath 2025.2.1
  wpimath_libs 
  wpimath_include_path
)
FETCH_MAVEN(release apriltag apriltag 2025.2.1
  apriltag_libs 
  apriltag_include_path
)

add_library(gtsam-localizer
  src/localizer.cpp
  src/TagModel.cpp
  src/gtsam_utils.cpp
  src/config.cpp
  src/camera_listener.cpp
  src/odom_listener.cpp
  src/data_publisher.cpp
  src/config_listener.cpp
)

message("Adding includes: ${gtsam_include_path} ${wpimath_include_path} ${wpiutil_include_path} ${apriltag_include_path} ")

target_include_directories(gtsam-localizer SYSTEM PRIVATE
  ${gtsam_include_path}
  ${wpimath_include_path}
  ${wpiutil_include_path}
  ${apriltag_include_path}
)
target_link_libraries(gtsam-localizer PUBLIC
  Eigen3::Eigen
  ${gtsam_libs}
  ${wpiutil_libs}
  ${wpimath_libs}
  ${apriltag_libs}
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
