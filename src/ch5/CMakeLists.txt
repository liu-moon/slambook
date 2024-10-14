cmake_minimum_required(VERSION 3.8)
project(ch5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(imageBasics src/imageBasics.cpp)
ament_target_dependencies(imageBasics rclcpp OpenCV cv_bridge)

add_executable(undistortImage src/undistortImage.cpp)
ament_target_dependencies(undistortImage rclcpp OpenCV cv_bridge)

# target_link_libraries(imageBasics ${OpenCV_LIBRARIES})

install(TARGETS
  imageBasics undistortImage
  DESTINATION lib/${PROJECT_NAME}
)