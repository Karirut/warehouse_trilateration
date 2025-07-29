//
// ROS2 Node for azure_kinect_ros2 driver
// 

//
#include <string>

// Library headers
//
#include <k4a/k4a.h>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>

// Project headers
//
#include "azure_kinect_ros2_driver/k4a_ros_device.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  // Instantiate the K4A node
  auto node = std::make_shared<K4AROS2Device>();
  executor.add_node(node);

  k4a_result_t result = node->startCameras();

  if (result != K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to start cameras");
    return -1;
  }

  result = node->startImu();
  if (result != K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to start IMU");
    return -2;
  }

  RCLCPP_INFO(node->get_logger(), "K4A Started");

  if (result == K4A_RESULT_SUCCEEDED)
  {
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "ROS Exit Started");
  }

  node.reset();

  RCLCPP_INFO(node->get_logger(), "ROS Exit");

  rclcpp::shutdown();

  RCLCPP_INFO(node->get_logger(), "ROS Shutdown complete");

  return 0;

}
