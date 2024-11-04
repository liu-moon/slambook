cmake_minimum_required(VERSION 3.8)
project(ch6)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



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
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)  # 添加 glog 查找
find_package(Ceres REQUIRED)  # 使用 find_package 查找 Ceres
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED)


# 手动设置Ceres头文件和库路径
include_directories(/usr/local/include/ceres)
link_directories(/usr/local/lib)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# 引入 FetchContent 模块
# include(FetchContent)

# # 使用 FetchContent_Declare 来获取 g2o 的源码
# FetchContent_Declare(
#   g2o
#   GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
#   GIT_TAG        20200410_git
# )
# FetchContent_MakeAvailable(g2o)

# set(g2o_path  ${PROJECT_SOURCE_DIR}/../../3rdparty/g2o)
# message(WARNING "CMAKE_SOURCE_DIR: ${g2o_path}")


# include_directories(${g2o_path}/include)

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

add_executable(gaussNewton src/gaussNewton.cpp)
ament_target_dependencies(gaussNewton rclcpp Eigen3 OpenCV)



add_executable(ceresCurveFitting src/ceresCurveFitting.cpp)
ament_target_dependencies(ceresCurveFitting rclcpp Eigen3 OpenCV)

include_directories("/usr/local/g2o_debug/include")


add_executable(g2oCurveFitting src/g2oCurveFitting.cpp)
ament_target_dependencies(g2oCurveFitting rclcpp Eigen3 OpenCV)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS}
  /usr/local/g2o_debug/lib/libg2o_core_d.so
  /usr/local/g2o_debug/lib/libg2o_stuff_d.so)

# 链接Ceres库
target_link_libraries(ceresCurveFitting /usr/local/lib/libceres.so glog::glog)

install(TARGETS
  gaussNewton ceresCurveFitting g2oCurveFitting
  DESTINATION lib/${PROJECT_NAME}
)


