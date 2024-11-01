cmake_minimum_required(VERSION 3.8)
project(ch3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Debug")

# 只在 Debug 模式下使用 no optimization (-O0)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

add_executable(eigen_node src/eigenMatrix.cpp)
ament_target_dependencies(eigen_node rclcpp Eigen3)

add_executable(useGeometry src/useGeometry.cpp)
ament_target_dependencies(useGeometry rclcpp Eigen3)

add_executable(coordinateTransform src/coordinateTransform.cpp)
ament_target_dependencies(coordinateTransform rclcpp Eigen3)

add_executable(plotTrajectory src/plotTrajectory.cpp)
ament_target_dependencies(plotTrajectory rclcpp Eigen3 nav_msgs)

add_executable(plotTrajectory_arrow src/plotTrajectory_arrow.cpp)
ament_target_dependencies(plotTrajectory_arrow rclcpp Eigen3 nav_msgs visualization_msgs)


install(TARGETS
  eigen_node useGeometry coordinateTransform plotTrajectory plotTrajectory_arrow
  DESTINATION lib/${PROJECT_NAME})