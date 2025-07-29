// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros2_driver/k4a_ros_device_params.h"

// Library headers
//
#include <k4a/k4a.h>

k4a_result_t K4AROSDeviceParams::GetDeviceConfig(k4a_device_configuration_t* configuration,
                                                 rclcpp::Node* node)
{
  configuration->depth_delay_off_color_usec = 0;
  configuration->disable_streaming_indicator = false;

  long pWiredSync = node->get_parameter("wired_sync_mode").as_int();

  RCLCPP_INFO_STREAM(node->get_logger(), "Setting wired sync mode: " << pWiredSync);
  if (pWiredSync == 0)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
  }
  else if (pWiredSync == 1)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
  }
  else if (pWiredSync == 2)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
  }
  else
  {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid wired sync mode: " << pWiredSync);
      return K4A_RESULT_FAILED;
  }


  RCLCPP_INFO_STREAM( node->get_logger(), "Setting subordinate delay: " <<
    node->get_parameter("subordinate_delay_off_master_usec").as_int());

  configuration->subordinate_delay_off_master_usec = node->get_parameter("subordinate_delay_off_master_usec").as_int();

  if (!node->get_parameter("color_enabled").as_bool())
  {
    RCLCPP_INFO(node->get_logger(), "Disabling RGB Camera");

    configuration->color_resolution = K4A_COLOR_RESOLUTION_OFF;
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Setting RGB Camera Format: " <<
      node->get_parameter("color_format").as_string());

    if (node->get_parameter("color_format").as_string() == "jpeg")
    {
      configuration->color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    }
    else if (node->get_parameter("color_format").as_string()  == "bgra")
    {
      configuration->color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid RGB Camera Format: " <<
        node->get_parameter("color_format").as_string());
      return K4A_RESULT_FAILED;
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Setting RGB Camera Resolution: " <<
      node->get_parameter("color_resolution").as_string());

    if (node->get_parameter("color_resolution").as_string()  == "720P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_720P;
    }
    else if (node->get_parameter("color_resolution").as_string() == "1080P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1080P;
    }
    else if (node->get_parameter("color_resolution").as_string() == "1440P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1440P;
    }
    else if (node->get_parameter("color_resolution").as_string() == "1536P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1536P;
    }
    else if (node->get_parameter("color_resolution").as_string() == "2160P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_2160P;
    }
    else if (node->get_parameter("color_resolution").as_string() == "3072P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_3072P;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid RGB Camera Resolution: " <<
        node->get_parameter("color_resolution").as_string());
      return K4A_RESULT_FAILED;
    }
  }

  if (!node->get_parameter("depth_enabled").as_bool())
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Disabling Depth Camera");

    configuration->depth_mode = K4A_DEPTH_MODE_OFF;
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Setting Depth Camera Mode: " <<
      node->get_parameter("depth_mode").as_string());

    if (node->get_parameter("depth_mode").as_string() == "NFOV_2X2BINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    }
    else if (node->get_parameter("depth_mode").as_string()  == "NFOV_UNBINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    }
    else if (node->get_parameter("depth_mode").as_string()  == "WFOV_2X2BINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    }
    else if (node->get_parameter("depth_mode").as_string()  == "WFOV_UNBINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    }
    else if (node->get_parameter("depth_mode").as_string()  == "PASSIVE_IR")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid Depth Camera Mode: " <<
        node->get_parameter("depth_mode").as_string());
      return K4A_RESULT_FAILED;
    }
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Setting Camera FPS: " << node->get_parameter("fps").as_int());

  if (node->get_parameter("fps").as_int() == 5)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_5;
  }
  else if (node->get_parameter("fps").as_int() == 15)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_15;
  }
  else if (node->get_parameter("fps").as_int() == 30)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_30;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid Camera FPS: " << node->get_parameter("fps").as_int());
    return K4A_RESULT_FAILED;
  }

  // Ensure that if RGB and depth cameras are enabled, we ask for synchronized frames
  if (node->get_parameter("depth_enabled").as_bool() && node->get_parameter("color_enabled").as_bool())
  {
    configuration->synchronized_images_only = true;
  }
  else
  {
    configuration->synchronized_images_only = false;
  }

  // Ensure that the "point_cloud" option is not used with passive IR mode, since they are incompatible
  if (node->get_parameter("point_cloud").as_bool() && (configuration->depth_mode == K4A_DEPTH_MODE_PASSIVE_IR))
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Incompatible options: cannot generate point cloud if depth camera is using PASSIVE_IR mode.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that point_cloud is enabled if using rgb_point_cloud
  if (node->get_parameter("rgb_point_cloud").as_bool() && !node->get_parameter("point_cloud").as_bool())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Incompatible options: cannot generate RGB point cloud if point_cloud is not enabled.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that color camera is enabled when generating a color point cloud
  if (node->get_parameter("rgb_point_cloud").as_bool() && !node->get_parameter("color_enabled").as_bool())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Incompatible options: cannot generate RGB point cloud if color camera is not enabled.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that color image contains RGB pixels instead of compressed JPEG data.
  if (node->get_parameter("rgb_point_cloud").as_bool() &&
      node->get_parameter("color_format").as_string() == "jpeg")
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Incompatible options: cannot generate RGB point cloud if color format is JPEG.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that target IMU rate is feasible
  if (node->get_parameter("imu_rate_target").as_int() == 0)
  {
    node->set_parameter(rclcpp::Parameter("imu_rate_target", IMU_MAX_RATE));
    RCLCPP_INFO_STREAM(node->get_logger(), "Using default IMU rate. Setting to maximum: " << IMU_MAX_RATE << " Hz.");
  }

  if (node->get_parameter("imu_rate_target").as_int() < 0 ||
      node->get_parameter("imu_rate_target").as_int() > IMU_MAX_RATE)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Incompatible options: desired IMU rate of " <<
      node->get_parameter("imu_rate_target").as_int() << "is not supported.");
    return K4A_RESULT_FAILED;
  }

  int div = IMU_MAX_RATE / node->get_parameter("imu_rate_target").as_int();
  float imu_rate_rounded = IMU_MAX_RATE / div;
  // Since we will throttle the IMU by averaging div samples together, this is the
  // achievable rate when rounded to the nearest whole number div.

  RCLCPP_INFO_STREAM(node->get_logger(), "Setting Target IMU rate to " << imu_rate_rounded <<
    " (desired: " << node->get_parameter("imu_rate_target").as_int() << ")");

  return K4A_RESULT_SUCCEEDED;
}
