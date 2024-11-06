cmake_minimum_required(VERSION 3.8)
project(ch9)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Debug")

# 只在 Debug 模式下使用 no optimization (-O0)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Ceres REQUIRED)  # 使用 find_package 查找 Ceres
# 手动设置Ceres头文件和库路径
include_directories(/usr/local/include/ceres)
link_directories(/usr/local/lib)


find_package(Eigen3 REQUIRED)
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

add_library(bal_common ./src/common.cpp)

add_executable(bundle_adjustment_ceres src/bundle_adjustment_ceres.cpp)
ament_target_dependencies(bundle_adjustment_ceres rclcpp Eigen3)

# 链接Ceres库
target_link_libraries(bundle_adjustment_ceres /usr/local/lib/libceres.so glog::glog bal_common)

install(TARGETS
  bundle_adjustment_ceres
  DESTINATION lib/${PROJECT_NAME}
)