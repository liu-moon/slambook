cmake_minimum_required(VERSION 3.8)
project(ch5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Debug")

# 只在 Debug 模式下使用 no optimization (-O0)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

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


add_executable(stereoVision src/stereoVision.cpp)
ament_target_dependencies(stereoVision rclcpp OpenCV cv_bridge Eigen3 sensor_msgs)

add_executable(jointMap src/jointMap.cpp)
ament_target_dependencies(jointMap rclcpp OpenCV cv_bridge Eigen3 sensor_msgs)

install(TARGETS
  imageBasics undistortImage stereoVision jointMap
  DESTINATION lib/${PROJECT_NAME}
)