cmake_minimum_required(VERSION 3.8)
project(ch4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(nav_msgs REQUIRED)
set(Sophus_LIBRARIES libSophus.so)

include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${Sophus_INCLUDE_DIRS})
# Sophus
include_directories("/usr/local/include")

# 添加库路径（手动指定）
# link_directories("/usr/local/lib")

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

add_executable(useSophus src/useSophus.cpp)
ament_target_dependencies(useSophus rclcpp Eigen3)

include_directories(/usr/local/include/sophus)



add_executable(trajectoryError src/trajectoryError.cpp)
ament_target_dependencies(trajectoryError rclcpp Eigen3 nav_msgs)

# 链接库文件
# target_link_libraries(useSophus ${Sophus_LIBRARIES})

install(TARGETS
  useSophus trajectoryError
  DESTINATION lib/${PROJECT_NAME})