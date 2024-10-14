# README

## 创建功能包

``` shell
cd slambook/src
ros2 pkg create ch --build-type ament_cmake --dependencies rclcpp
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

