project(gtsam-playground)

add_subdirectory(gtsam)

add_executable(gtsam-test src/test_fake_tags.cpp)
target_link_libraries(gtsam-test 
  gtsam
)
