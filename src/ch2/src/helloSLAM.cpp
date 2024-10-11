#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);

  // Create a node
  auto node = rclcpp::Node::make_shared("hello_slam_node");

  // Log the message
  RCLCPP_INFO(node->get_logger(), "Hello SLAM!");

  // Spin the node to process callbacks
  rclcpp::spin(node);

  // Shutdown the ROS2 system
  rclcpp::shutdown();
  
  return 0;
}
