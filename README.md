# README

## 创建功能包

``` shell
cd slambook/src
ros2 pkg create chx --build-type ament_cmake --dependencies rclcpp
```

## 编译节点

``` shell
cd slambook
colcon build
```

## source环境

``` shell
source install/setup.bash
```

## 运行节点

``` shell
ros2 run chx xxx
```

## 基本模板

``` c++
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("MyNode")
  {
    
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
```

## 打开rviz2

``` shell
ros2 run rviz2 rviz2
```

## 安装ceres

### 安装依赖库

``` shell
sudo apt update
sudo apt install cmake libgoogle-glog-dev libgflags-dev libeigen3-dev libsuitesparse-dev libabsl-dev

```