cmake_minimum_required(VERSION 3.8)
project(ch7)

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
find_package(Sophus REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("/usr/local/g2o_debug/include")



add_compile_options(-msse4.2)


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

add_executable(orb_cv src/orb_cv.cpp)
ament_target_dependencies(orb_cv OpenCV rclcpp cv_bridge)

add_executable(orb_self src/orb_self.cpp)
ament_target_dependencies(orb_self OpenCV rclcpp cv_bridge)

add_executable(pose_estimation_2d2d src/pose_estimation_2d2d.cpp)
ament_target_dependencies(pose_estimation_2d2d OpenCV rclcpp cv_bridge)


add_executable(triangulation src/triangulation.cpp)
ament_target_dependencies(triangulation OpenCV rclcpp cv_bridge)



add_executable(pose_estimation_3d2d src/pose_estimation_3d2d.cpp)
ament_target_dependencies(pose_estimation_3d2d OpenCV rclcpp cv_bridge Eigen3)

target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS}
  /usr/local/g2o_debug/lib/libg2o_core_d.so
  /usr/local/g2o_debug/lib/libg2o_stuff_d.so)
  
set_target_properties(pose_estimation_3d2d PROPERTIES
  BUILD_RPATH "/usr/local/g2o_debug/lib"
  INSTALL_RPATH "/usr/local/g2o_debug/lib"
)


add_executable(pose_estimation_3d3d src/pose_estimation_3d3d.cpp)
ament_target_dependencies(pose_estimation_3d3d OpenCV rclcpp cv_bridge Eigen3)

target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBS}
  /usr/local/g2o_debug/lib/libg2o_core_d.so
  /usr/local/g2o_debug/lib/libg2o_stuff_d.so)
  
set_target_properties(pose_estimation_3d3d PROPERTIES
  BUILD_RPATH "/usr/local/g2o_debug/lib"
  INSTALL_RPATH "/usr/local/g2o_debug/lib"
)

install(TARGETS
  orb_cv orb_self pose_estimation_2d2d triangulation pose_estimation_3d2d pose_estimation_3d3d
  DESTINATION lib/${PROJECT_NAME}
)