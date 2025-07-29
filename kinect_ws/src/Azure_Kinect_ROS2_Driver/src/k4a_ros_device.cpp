// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros2_driver/k4a_ros_device.h"


// System headers
//
#include <thread>

// Library headers
//
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <k4a/k4a.hpp>

//#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


// Project headers
//
#include "azure_kinect_ros2_driver/k4a_ros_types.h"



using namespace sensor_msgs;
using namespace image_transport;
using namespace std;


K4AROS2Device::K4AROS2Device()
    : Node("k4a_ros2_node"),
      last_capture_time_usec_(0),
      qos_(1),
      last_imu_time_usec_(0),
      process_cloud_(false),
      imu_stream_end_of_file_(false)
{

  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing " << this->get_name() << "...");

  // Collect ROS parameters from the param server or from the command line

  calibration_data_ = std::make_unique<K4ACalibrationTransformData>(this);


  // Declare the params
  std::string pSensorSn = this->declare_parameter<std::string>("sensor_sn", "");
  this->declare_parameter<bool>("depth_enabled", true);
  this->declare_parameter<std::string>("depth_mode", "NFOV_UNBINNED");
  this->declare_parameter<bool>("color_enabled", true);
  std::string pColorFormat = this->declare_parameter<std::string>("color_format", "bgra");
  this->declare_parameter<std::string>("color_resolution", "1536P");
  this->declare_parameter<int>("fps", 30);
  this->declare_parameter<bool>("point_cloud", false); //false
  this->declare_parameter<bool>("rgb_point_cloud", false); //false
  this->declare_parameter<bool>("point_cloud_in_depth_frame", true);
  this->declare_parameter<std::string>("tf_prefix", std::string());
  std::string pRecordingFile = this->declare_parameter<std::string>("recording_file", "");
  this->declare_parameter<bool>("recording_loop_enabled", false);
  this->declare_parameter<bool>("body_tracking_enabled", false);
  this->declare_parameter<int>("imu_rate_target", 100);
  this->declare_parameter<bool>("rescale_ir_to_mono8", false);
  this->declare_parameter<float>("ir_mono8_scaling_factor", 1.0f);;
  this->declare_parameter<int>("wired_sync_mode", 0);
  this->declare_parameter<int>("subordinate_delay_off_master_usec", 0);

  // TODO: QoS
  //int pQosReliability = this->declare_parameter<int>("qos_reliability", 1);
  //int pQosDurability = this->declare_parameter<int>("qos_durability", 1);

  if (pRecordingFile != "")
  {
    // Replace the first "~"
    std::string home_dir = getenv("HOME");
    std::size_t pos = pRecordingFile.find("~");
    if (pos != std::string::npos)
    {
      pRecordingFile.replace(pos, 1, home_dir);
    }

    RCLCPP_INFO(this->get_logger(), "Node is started in playback mode");
    RCLCPP_INFO_STREAM(this->get_logger(), "Try to open recording file " << pRecordingFile);

    // Open recording file and print its length
    k4a_playback_handle_ = k4a::playback::open(pRecordingFile.c_str());
    auto recording_length = k4a_playback_handle_.get_recording_length();
    RCLCPP_INFO_STREAM(this->get_logger(), "Successfully opened recording file. Recording is " << recording_length.count() / 1000000
                                                                         << " seconds long");

    // Get the recordings configuration to overwrite node parameters
    k4a_record_configuration_t record_config = k4a_playback_handle_.get_record_configuration();

    // Overwrite fps param with recording configuration for a correct loop rate in the frame publisher thread
    switch (record_config.camera_fps)
    {
      case K4A_FRAMES_PER_SECOND_5:
        this->set_parameter(rclcpp::Parameter("fps", 5));
        break;
      case K4A_FRAMES_PER_SECOND_15:
        this->set_parameter(rclcpp::Parameter("fps", 15));
        break;
      case K4A_FRAMES_PER_SECOND_30:
        this->set_parameter(rclcpp::Parameter("fps", 30));
        break;
      default:
        break;
    };

    // Disable color if the recording has no color track
    //if (params_.color_enabled && !record_config.color_track_enabled)
    if (this->get_parameter("color_enabled").as_bool() && !record_config.color_track_enabled)
    {
      RCLCPP_WARN(this->get_logger(), "Disabling color and rgb_point_cloud because recording has no color track");
      this->set_parameter(rclcpp::Parameter("color_enabled", false));
      this->set_parameter(rclcpp::Parameter("point_cloud", false)); //false
    }
    // This is necessary because at the moment there are only checks in place which use BgraPixel size
    else if (this->get_parameter("color_enabled").as_bool() && record_config.color_track_enabled)
    {
      if (pColorFormat == "jpeg" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_MJPG)
      {
        RCLCPP_FATAL(this->get_logger(), "Converting color images to K4A_IMAGE_FORMAT_COLOR_MJPG is not supported.");
        rclcpp::shutdown();
        return;
      }
      if (pColorFormat == "bgra" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
      {
        k4a_playback_handle_.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
      }
    }

    // Disable depth if the recording has neither ir track nor depth track
    if (!record_config.ir_track_enabled && !record_config.depth_track_enabled)
    {
      if (this->get_parameter("depth_enabled").as_bool())
      {
        RCLCPP_WARN(this->get_logger(), "Disabling depth because recording has neither ir track nor depth track");
        this->set_parameter(rclcpp::Parameter("depth_enabled", false));
      }
    }

    // Disable depth if the recording has no depth track
    if (!record_config.depth_track_enabled)
    {
      RCLCPP_WARN(this->get_logger(), "No depth track in recording");
      if (this->get_parameter("point_cloud").as_bool())
      {
        RCLCPP_WARN(this->get_logger(), "Disabling point cloud because recording has no depth track");
        this->set_parameter(rclcpp::Parameter("point_cloud", false)); //false
      }
      if (this->get_parameter("rgb_point_cloud").as_bool())
      {
        RCLCPP_WARN(this->get_logger(), "Disabling rgb point cloud because recording has no depth track");
        this->set_parameter(rclcpp::Parameter("rgb_point_cloud", false));
      }
    }
    RCLCPP_INFO(this->get_logger(), "Recording has depth track");

  }
  else
  {
    // Print all parameters
    RCLCPP_INFO(this->get_logger(), "K4A Parameters:");

    std::vector<std::string> param_names = {"sensor_sn", "depth_enabled", "depth_mode","color_enabled",
                                            "color_format", "color_resolution","fps", "point_cloud",
                                            "rgb_point_cloud", "point_cloud_in_depth_frame", "tf_prefix",
                                            "recording_file", "recording_loop_enabled", "imu_rate_target",
                                            "wired_sync_mode", "subordinate_delay_off_master_usec"};
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    for (auto &param : params)
    {
      RCLCPP_INFO(this->get_logger(), "param name: %s, value: %s",
                  param.get_name().c_str(), param.value_to_string().c_str());
    }


    // Setup the K4A device
    uint32_t k4a_device_count = k4a::device::get_installed_count();

    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << k4a_device_count << " sensors");

    if (pSensorSn != "")
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Searching for sensor with serial number: " << pSensorSn);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "No serial number provided: picking first sensor");
      RCLCPP_WARN_EXPRESSION(this->get_logger(), k4a_device_count > 1, "Multiple sensors connected! Picking first sensor.");
    }

    for (uint32_t i = 0; i < k4a_device_count; i++)
    {
      k4a::device device;
      try
      {
        device = k4a::device::open(i);
      }
      catch (exception)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open K4A device at index " << i);
        continue;
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "K4A[" << i << "] : " << device.get_serialnum());

      // Try to match serial number
      if (pSensorSn!= "")
      {
        if (device.get_serialnum() == pSensorSn)
        {
          k4a_device_ = std::move(device);
          break;
        }
      }
      // Pick the first device
      else if (i == 0)
      {
        k4a_device_ = std::move(device);
        break;
      }
    }

    if (!k4a_device_)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open a K4A device. Cannot continue.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "K4A Serial Number: " << k4a_device_.get_serialnum());

    k4a_hardware_version_t version_info = k4a_device_.get_version();

    RCLCPP_INFO(this->get_logger(), "RGB Version: %d.%d.%d", version_info.rgb.major, version_info.rgb.minor, version_info.rgb.iteration);

    RCLCPP_INFO(this->get_logger(), "Depth Version: %d.%d.%d", version_info.depth.major, version_info.depth.minor,
             version_info.depth.iteration);

    RCLCPP_INFO(this->get_logger(), "Audio Version: %d.%d.%d", version_info.audio.major, version_info.audio.minor,
             version_info.audio.iteration);

    RCLCPP_INFO(this->get_logger(), "Depth Sensor Version: %d.%d.%d", version_info.depth_sensor.major, version_info.depth_sensor.minor,
             version_info.depth_sensor.iteration);
  }


  // TODO: QoS Params
  qos_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  std::string topic_prefix = "k4a/";


  // Register our topics
  if (pColorFormat == "jpeg")
  {
    // JPEG images are directly published on 'rgb/image_raw/compressed' so that
    // others can subscribe to 'rgb/image_raw' with compressed_image_transport.
    // This technique is described in:
    // http://wiki.ros.org/compressed_image_transport#Publishing_compressed_images_directly

    // I guess CompressedImage cannot use CameraPublisher. It needs its own separate publishers for the image
    // and the camera_info. https://answers.ros.org/question/385599/how-to-publish-a-compressedimage-in-ros2-foxy/
    rgb_jpeg_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "rgb/image_raw/compressed", qos_);

    rgb_cam_info_jpeg_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "rgb/camera_info", qos_);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Advertised on topic: " << rgb_jpeg_publisher_->get_topic_name());
  }
  else if (pColorFormat == "bgra")
  {
    rgb_raw_publisher_ = image_transport::create_camera_publisher(this,
                                                                  topic_prefix + "rgb/image_raw",
                                                                  qos_.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Advertised on topic: " << rgb_raw_publisher_.getTopic());
  }

  depth_raw_publisher_ = image_transport::create_camera_publisher(this,
                                                                  topic_prefix + "depth/image_raw",
                                                                  qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << depth_raw_publisher_.getTopic());

  depth_rect_publisher_ = image_transport::create_camera_publisher(this,
                                                                   topic_prefix + "depth_to_rgb/image_raw",
                                                                   qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << depth_rect_publisher_.getTopic());
  rgb_rect_publisher_ = image_transport::create_camera_publisher(this,
                                                                 topic_prefix + "rgb_to_depth/image_raw",
                                                                 qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << rgb_rect_publisher_.getTopic());
  ir_raw_publisher_ = image_transport::create_camera_publisher(this,
                                                               topic_prefix + "ir/image_raw",
                                                               qos_.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Advertised on topic: " << ir_raw_publisher_.getTopic());

  imu_orientation_publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic_prefix + "imu",
                                                                       qos_);
  RCLCPP_INFO_STREAM(get_logger(),
                     "Advertised on topic: " << imu_orientation_publisher_->get_topic_name());

  if (this->get_parameter("point_cloud").as_bool() || this->get_parameter("rgb_point_cloud").as_bool()) {
    process_cloud_ = true;
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_prefix + "points2",
                                                                            qos_);
  }

}

K4AROS2Device::~K4AROS2Device()
{
  // Start tearing down the publisher threads
  running_ = false;

  // Join the publisher thread
  RCLCPP_INFO(this->get_logger(), "Joining camera publisher thread");
  frame_publisher_thread_.join();
  RCLCPP_INFO(this->get_logger(), "Camera publisher thread joined");

  // Join the publisher thread
  RCLCPP_INFO(this->get_logger(), "Joining IMU publisher thread");
  imu_publisher_thread_.join();
  RCLCPP_INFO(this->get_logger(), "IMU publisher thread joined");

  stopCameras();
  stopImu();

  if (k4a_playback_handle_)
  {
    k4a_playback_handle_.close();
  }
}

k4a_result_t K4AROS2Device::startCameras()
{
  k4a_device_configuration_t k4a_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  k4a_result_t result = K4AROSDeviceParams::GetDeviceConfig(&k4a_configuration, this);

  if (k4a_device_)
  {
    if (result != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate a device configuration. Not starting camera!");
      return result;
    }

    // Now that we have a proposed camera configuration, we can
    // initialize the class which will take care of device calibration information
    calibration_data_->initialize(k4a_device_, k4a_configuration.depth_mode, k4a_configuration.color_resolution);
  }
  else if (k4a_playback_handle_)
  {
    // initialize the class which will take care of device calibration information from the playback_handle
    calibration_data_->initialize(k4a_playback_handle_);
  }


  if (k4a_device_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "STARTING CAMERAS");
    k4a_device_.start_cameras(&k4a_configuration);
  }

  // Cannot assume the device timestamp begins increasing upon starting the cameras.
  // If we set the time base here, depending on the machine performance, the new timestamp
  // would lag the value of ros::Time::now() by at least 0.5 secs which is much larger than
  // the real transmission delay as can be observed using the rqt_plot tool.
  // start_time_ = ros::Time::now();

  // Prevent the worker thread from exiting immediately
  running_ = true;

  // Start the thread that will poll the cameras and publish frames
  frame_publisher_thread_ = thread(&K4AROS2Device::framePublisherThread, this);

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::startImu()
{
  if (k4a_device_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "STARTING IMU");
    k4a_device_.start_imu();
  }

  RCLCPP_DEBUG(this->get_logger(), "IMU started. Kicking off IMU publisher thread.");

  // Start the IMU publisher thread
  imu_publisher_thread_ = thread(&K4AROS2Device::imuPublisherThread, this);


  return K4A_RESULT_SUCCEEDED;
}

void K4AROS2Device::stopCameras()
{
  if (k4a_device_)
  {
    // Stop the K4A SDK
    RCLCPP_INFO(this->get_logger(), "Stopping K4A device");
    k4a_device_.stop_cameras();
    RCLCPP_INFO(this->get_logger(), "K4A device stopped");
  }
}

void K4AROS2Device::stopImu()
{
  if (k4a_device_)
  {
    k4a_device_.stop_imu();
  }
}

k4a_result_t K4AROS2Device::getDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& depth_image,
                                         bool rectified = false)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render depth frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    calibration_data_->k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                      &calibration_data_->transformed_depth_image_);

    return renderDepthToROS(depth_image, calibration_data_->transformed_depth_image_);
  }

  return renderDepthToROS(depth_image, k4a_depth_frame);
}

k4a_result_t K4AROS2Device::renderDepthToROS(std::shared_ptr<sensor_msgs::msg::Image>& depth_image, k4a::image& k4a_depth_frame)
{
  cv::Mat depth_frame_buffer_mat(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1,
                                 k4a_depth_frame.get_buffer());
  cv::Mat new_image(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_32FC1);

  depth_frame_buffer_mat.convertTo(new_image, CV_32FC1, 1.0 / 1000.0f);

  depth_image =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_32FC1, new_image).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getIrFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& ir_image)
{
  k4a::image k4a_ir_frame = capture.get_ir_image();

  if (!k4a_ir_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render IR frame: no frame");
    return K4A_RESULT_FAILED;
  }

  return renderIrToROS(ir_image, k4a_ir_frame);
}

k4a_result_t K4AROS2Device::renderIrToROS(std::shared_ptr<sensor_msgs::msg::Image>& ir_image, k4a::image& k4a_ir_frame)
{
  cv::Mat ir_buffer_mat(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_16UC1,
                        k4a_ir_frame.get_buffer());

  // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
  if (this->get_parameter("rescale_ir_to_mono8").as_bool())
  {
    cv::Mat new_image(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_8UC1);
    // Use a scaling factor to re-scale the image. If using the illuminators, a value of 1 is appropriate.
    // If using PASSIVE_IR, then a value of 10 is more appropriate; k4aviewer does a similar conversion.
    ir_buffer_mat.convertTo(new_image, CV_8UC1, this->get_parameter("ir_mono8_scaling_factor").as_double());
    ir_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_image).toImageMsg();
  }
  else
  {
    ir_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getJpegRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::CompressedImage> jpeg_image)
{
  k4a::image k4a_jpeg_frame = capture.get_color_image();

  if (!k4a_jpeg_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render Jpeg frame: no frame");
    return K4A_RESULT_FAILED;
  }

  const uint8_t* jpeg_frame_buffer = k4a_jpeg_frame.get_buffer();
  jpeg_image->format = "bgra8; jpeg compressed bgr8";
  jpeg_image->data.assign(jpeg_frame_buffer, jpeg_frame_buffer + k4a_jpeg_frame.get_size());
  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getRbgFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& rgb_image,
                                       bool rectified = false)
{
  k4a::image k4a_bgra_frame = capture.get_color_image();

  if (!k4a_bgra_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render BGRA frame: no frame");
    return K4A_RESULT_FAILED;
  }

  size_t color_image_size =
      static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) * sizeof(BgraPixel);

  if (k4a_bgra_frame.get_size() != color_image_size)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid k4a_bgra_frame returned from K4A");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    k4a::image k4a_depth_frame = capture.get_depth_image();

    calibration_data_->k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
                                                                      &calibration_data_->transformed_rgb_image_);


    return renderBGRA32ToROS(rgb_image, calibration_data_->transformed_rgb_image_);
  }

  return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}

// Helper function that renders any BGRA K4A frame to a ROS ImagePtr. Useful for rendering intermediary frames
// during debugging of image processing functions
k4a_result_t K4AROS2Device::renderBGRA32ToROS(std::shared_ptr<sensor_msgs::msg::Image>& rgb_image, k4a::image& k4a_bgra_frame)
{
  cv::Mat rgb_buffer_mat(k4a_bgra_frame.get_height_pixels(), k4a_bgra_frame.get_width_pixels(), CV_8UC4,
                         k4a_bgra_frame.get_buffer());

  rgb_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGRA8, rgb_buffer_mat).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getRgbPointCloudInDepthFrame(const k4a::capture& capture,
                                                         std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud)
{
  const k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  const k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // Transform color image into the depth camera frame:
  calibration_data_->k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
                                                                    &calibration_data_->transformed_rgb_image_);

  // Tranform depth image to point cloud
  calibration_data_->k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_->point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  this->printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);

  return fillColorPointCloud(calibration_data_->point_cloud_image_, calibration_data_->transformed_rgb_image_,
                             point_cloud);
}

k4a_result_t K4AROS2Device::getRgbPointCloudInRgbFrame(const k4a::capture& capture,
                                                       std::shared_ptr<sensor_msgs::msg::PointCloud2>  point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // transform depth image into color camera geometry
  calibration_data_->k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                    &calibration_data_->transformed_depth_image_);

  // Tranform depth image to point cloud (note that this is now from the perspective of the color camera)
  calibration_data_->k4a_transformation_.depth_image_to_point_cloud(
      calibration_data_->transformed_depth_image_, K4A_CALIBRATION_TYPE_COLOR, &calibration_data_->point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->rgb_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_bgra_frame.get_device_timestamp());
  this->printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);

  return fillColorPointCloud(calibration_data_->point_cloud_image_, k4a_bgra_frame, point_cloud);
}

k4a_result_t K4AROS2Device::getPointCloud(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>  point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot render point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  point_cloud->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  this->printTimestampDebugMessage("Point cloud", point_cloud->header.stamp);

  // Tranform depth image to point cloud
  calibration_data_->k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_->point_cloud_image_);

  return fillPointCloud(calibration_data_->point_cloud_image_, point_cloud);
}

k4a_result_t K4AROS2Device::fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                                std::shared_ptr<sensor_msgs::msg::PointCloud2>&  point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();
  const size_t pixel_count = color_image.get_size() / sizeof(BgraPixel);
  if (point_count != pixel_count)
  {
    RCLCPP_WARN(this->get_logger(), "Color and depth image sizes do not match!");
    return K4A_RESULT_FAILED;
  }

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud, "b");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());
  const uint8_t* color_buffer = color_image.get_buffer();

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    // Z in image frame:
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);
    // Alpha value:
    uint8_t a = color_buffer[4 * i + 3];
    if (z <= 0.0f || a == 0)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      *iter_r = *iter_g = *iter_b = 0;
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;

      *iter_r = color_buffer[4 * i + 2];
      *iter_g = color_buffer[4 * i + 1];
      *iter_b = color_buffer[4 * i + 0];
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::fillPointCloud(const k4a::image& pointcloud_image,
                                           std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
  {
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

    if (z <= 0.0f)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROS2Device::getImuFrame(const k4a_imu_sample_t& sample, std::shared_ptr<sensor_msgs::msg::Imu> imu_msg)
{
  imu_msg->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->imu_frame_;
  imu_msg->header.stamp = timestampToROS(sample.acc_timestamp_usec);
  this->printTimestampDebugMessage("IMU", imu_msg->header.stamp);

  // The correct convention in ROS is to publish the raw sensor data, in the
  // sensor coordinate frame. Do that here.
  imu_msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  imu_msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  imu_msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  imu_msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  imu_msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  imu_msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  imu_msg->orientation_covariance[0] = -1.0;

  return K4A_RESULT_SUCCEEDED;
}


void K4AROS2Device::framePublisherThread()
{
  rclcpp::Rate loop_rate(this->get_parameter("fps").as_int());

  k4a_wait_result_t wait_result;
  k4a_result_t result;

  std::shared_ptr<sensor_msgs::msg::CameraInfo> rgb_raw_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  std::shared_ptr<sensor_msgs::msg::CameraInfo> depth_raw_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  std::shared_ptr<sensor_msgs::msg::CameraInfo> rgb_rect_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  std::shared_ptr<sensor_msgs::msg::CameraInfo> depth_rect_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  std::shared_ptr<sensor_msgs::msg::CameraInfo> ir_raw_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();

  rclcpp::Time capture_time;

  k4a::capture capture;

  calibration_data_->getDepthCameraInfo(depth_raw_camera_info);
  calibration_data_->getRgbCameraInfo(rgb_raw_camera_info);
  calibration_data_->getDepthCameraInfo(rgb_rect_camera_info);
  calibration_data_->getRgbCameraInfo(depth_rect_camera_info);
  calibration_data_->getDepthCameraInfo(ir_raw_camera_info);

  //while (running_ && rclcpp::ok() && !rclcpp::isShuttingDown())
  RCLCPP_INFO(this->get_logger(), "Starting image grab loop...");
  while (running_ && rclcpp::ok())
  {

    rclcpp::Time cycle_start_time = this->now();

    if (k4a_device_)
    {
      RCLCPP_DEBUG(this->get_logger(), "Processing k4a device loop");
      // TODO: consider appropriate capture timeout based on camera framerate
      if (!k4a_device_.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
      {
        RCLCPP_FATAL(this->get_logger(), "Failed to poll cameras: node cannot continue.");
        rclcpp::shutdown();
        return;
      }
      else
      {

        if (this->get_parameter("depth_enabled").as_bool())
        {
          // Update the timestamp offset based on the difference between the system timestamp (i.e., arrival at USB bus)
          // and device timestamp (i.e., hardware clock at exposure start).
          updateTimestampOffset(capture.get_ir_image().get_device_timestamp(),
                                capture.get_ir_image().get_system_timestamp());
        }
        else if (this->get_parameter("color_enabled").as_bool())
        {
          updateTimestampOffset(capture.get_color_image().get_device_timestamp(),
                                capture.get_color_image().get_system_timestamp());
        }
      }
    }
    else if (k4a_playback_handle_)
    {
      std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
      if (!k4a_playback_handle_.get_next_capture(&capture))
      {
        // rewind recording if looping is enabled
        if (this->get_parameter("recording_loop_enabled").as_bool())
        {
          k4a_playback_handle_.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
          k4a_playback_handle_.get_next_capture(&capture);
          imu_stream_end_of_file_ = false;
          last_imu_time_usec_ = 0;
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Recording reached end of file. Node will not continue.");
          rclcpp::shutdown();
          return;
        }
      }

      last_capture_time_usec_ = getCaptureTimestamp(capture).count();
    }

    // Instantiate messages to be published
    std::shared_ptr<sensor_msgs::msg::CompressedImage> rgb_jpeg_frame =
        std::shared_ptr<sensor_msgs::msg::CompressedImage>();

    std::shared_ptr<sensor_msgs::msg::Image> ir_raw_frame =
        std::make_shared<sensor_msgs::msg::Image>();

    std::shared_ptr<sensor_msgs::msg::Image> rgb_raw_frame =
        std::shared_ptr<sensor_msgs::msg::Image>();

    std::shared_ptr<sensor_msgs::msg::Image> rgb_rect_frame =
        std::shared_ptr<sensor_msgs::msg::Image>();

    std::shared_ptr<sensor_msgs::msg::Image> depth_raw_frame =
        std::shared_ptr<sensor_msgs::msg::Image>();

    std::shared_ptr<sensor_msgs::msg::Image> depth_rect_frame =
        std::shared_ptr<sensor_msgs::msg::Image>();

    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud =
        std::make_shared<sensor_msgs::msg::PointCloud2>();


    if (this->get_parameter("depth_enabled").as_bool())
    {
      // Only do compute if we have subscribers
      // Only create ir frame when we are using a device or we have an ir image.
      // Recordings may not have synchronized captures. For unsynchronized captures without ir image skip ir frame.
      if ((ir_raw_publisher_.getNumSubscribers() > 0) && (k4a_device_ || capture.get_ir_image() != nullptr))
      {
        // IR images are available in all depth modes
        result = getIrFrame(capture, ir_raw_frame);

        if (result != K4A_RESULT_SUCCEEDED)
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get raw IR frame");
          rclcpp::shutdown();
          return;
        }
        else if (result == K4A_RESULT_SUCCEEDED)
        {
          capture_time = timestampToROS(capture.get_ir_image().get_device_timestamp());
          this->printTimestampDebugMessage("IR image", capture_time);

          // Re-sychronize the timestamps with the capture timestamp
          ir_raw_camera_info->header.stamp = capture_time;
          ir_raw_frame->header.stamp = capture_time;
          ir_raw_frame->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;

          ir_raw_publisher_.publish(ir_raw_frame, ir_raw_camera_info);
        }
      }

      // Depth images are not available in PASSIVE_IR mode
      if (calibration_data_->k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR)
      {
        // Only create depth frame when we are using a device or we have an depth image.
        // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip depth
        // frame.
        if ((depth_raw_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || capture.get_depth_image() != nullptr))
        {
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Processing depth raw subscription(s): " << depth_raw_publisher_.getNumSubscribers());

          result = getDepthFrame(capture, depth_raw_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get raw depth frame");
            rclcpp::shutdown();
            return;
          }
          else if (result == K4A_RESULT_SUCCEEDED)
          {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());
            this->printTimestampDebugMessage("Depth image", capture_time);

            // Re-sychronize the timestamps with the capture timestamp
            depth_raw_camera_info->header.stamp = capture_time;
            depth_raw_frame->header.stamp = capture_time;
            depth_raw_frame->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;

            depth_raw_publisher_.publish(depth_raw_frame,depth_raw_camera_info);
          }
        }

        // We can only rectify the depth into the color co-ordinates if the color camera is enabled!
        // Only create rect depth frame when we are using a device or we have an depth image.
        // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip rect
        // depth frame.
        if (this->get_parameter("color_enabled").as_bool() &&
            depth_rect_publisher_.getNumSubscribers() > 0 &&
            (k4a_device_ || capture.get_depth_image() != nullptr))
        {
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Processing depth rect subscription(s): " << depth_rect_publisher_.getNumSubscribers());

          result = getDepthFrame(capture, depth_rect_frame, true /* rectified */);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get rectifed depth frame");
            rclcpp::shutdown();
            return;
          }
          else if (result == K4A_RESULT_SUCCEEDED)
          {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());
            this->printTimestampDebugMessage("Depth image", capture_time);

            depth_rect_frame->header.stamp = capture_time;
            depth_rect_frame->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->rgb_camera_frame_;

            // Re-synchronize the header timestamps since we cache the camera calibration message
            depth_rect_camera_info->header.stamp = capture_time;

            depth_rect_publisher_.publish(depth_rect_frame, depth_rect_camera_info);
          }
        }


      }
    }

    if (this->get_parameter("color_enabled").as_bool())
    {
      // Only create rgb frame when we are using a device or we have a color image.
      // Recordings may not have synchronized captures. For unsynchronized captures without color image skip rgb frame.
      if (this->get_parameter("color_format").as_string() == "jpeg")
      {
        if (rgb_jpeg_publisher_->get_subscription_count()  > 0 &&
            (k4a_device_ || capture.get_color_image() != nullptr))
        {
          result = getJpegRgbFrame(capture, rgb_jpeg_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get Jpeg frame");
            rclcpp::shutdown();
            return;
          }

          capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
          this->printTimestampDebugMessage("Color image", capture_time);

          rgb_jpeg_frame->header.stamp = capture_time;
          rgb_jpeg_frame->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->rgb_camera_frame_;
          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_raw_camera_info->header.stamp = capture_time;

          rgb_jpeg_publisher_->publish(*rgb_jpeg_frame);
          rgb_cam_info_jpeg_publisher_->publish(*rgb_raw_camera_info);
        }
      }
      else if (this->get_parameter("color_format").as_string() == "bgra")
      {
        if (rgb_raw_publisher_.getNumSubscribers() > 0 &&
            (k4a_device_ || capture.get_color_image() != nullptr))
        {
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Processing RGB raw subscription(s): " << rgb_raw_publisher_.getNumSubscribers());

          result = getRbgFrame(capture, rgb_raw_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get RGB frame");
            rclcpp::shutdown();
            return;
          }

          capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
          this->printTimestampDebugMessage("Color image", capture_time);

          rgb_raw_frame->header.stamp = capture_time;
          rgb_raw_frame->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->rgb_camera_frame_;

          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_raw_camera_info->header.stamp = capture_time;

          rgb_raw_publisher_.publish(rgb_raw_frame, rgb_raw_camera_info);
        }

        // We can only rectify the color into the depth co-ordinates if the depth camera is enabled and processing depth
        // data Only create rgb rect frame when we are using a device or we have a synchronized image. Recordings may
        // not have synchronized captures. For unsynchronized captures image skip rgb rect frame.
        if (this->get_parameter("depth_enabled").as_bool() &&
            (calibration_data_->k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) &&
            rgb_rect_publisher_.getNumSubscribers() > 0 &&
            (k4a_device_ || (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
        {
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Processing RGB rect subscription(s): " << rgb_rect_publisher_.getNumSubscribers());
          result = getRbgFrame(capture, rgb_rect_frame, true /* rectified */);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get rectifed depth frame");
            rclcpp::shutdown();
            return;
          }

          capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
          this->printTimestampDebugMessage("Color image", capture_time);

          rgb_rect_frame->header.stamp = capture_time;
          rgb_rect_frame->header.frame_id = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;

          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_rect_camera_info->header.stamp = capture_time;

          rgb_rect_publisher_.publish(rgb_rect_frame,rgb_rect_camera_info);
        }
      }
    }


    // Only create pointcloud when we are using a device or we have a synchronized image.
    // Recordings may not have synchronized captures. In unsynchronized captures skip point cloud.
    if (process_cloud_ && pointcloud_publisher_->get_subscription_count() > 0 &&
        (k4a_device_ || (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
    {
    
      RCLCPP_DEBUG(this->get_logger(), "Processing point cloud...");
      if (this->get_parameter("rgb_point_cloud").as_bool())
      {
        if (this->get_parameter("point_cloud_in_depth_frame").as_bool())
        {
          result = getRgbPointCloudInDepthFrame(capture, point_cloud);
        }
        else
        {
          result = getRgbPointCloudInRgbFrame(capture, point_cloud);
        }

        if (result != K4A_RESULT_SUCCEEDED)
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get RGB Point Cloud");
          rclcpp::shutdown();
          return;
        }
      }
      else if (this->get_parameter("point_cloud").as_bool())
      {
        result = getPointCloud(capture, point_cloud);

        if (result != K4A_RESULT_SUCCEEDED)
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to get Point Cloud");
          rclcpp::shutdown();
          return;
        }
      }

      if (this->get_parameter("point_cloud").as_bool() || this->get_parameter("rgb_point_cloud").as_bool())
      {
        pointcloud_publisher_->publish(*point_cloud);
      }
    }
    rclcpp::Duration cycle_time = this->now() - cycle_start_time;

    // If the cycle took longer than the expected rate (1/fps)
    if (cycle_time > loop_rate.period())
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Image processing thread is running behind."
                                       << std::endl
                                       << "Expected max loop time: " << loop_rate.period().count() / 1000000000. << std::endl
                                       << "Actual loop time: " << cycle_time.seconds() << std::endl);
    }

    loop_rate.sleep();
  }
}

k4a_imu_sample_t K4AROS2Device::computeMeanIMUSample(const std::vector<k4a_imu_sample_t>& samples)
{
  // Compute mean sample
  // Using double-precision version of imu sample struct to avoid overflow
  k4a_imu_accumulator_t mean;
  for (auto imu_sample : samples)
  {
    mean += imu_sample;
  }
  float num_samples = samples.size();
  mean /= num_samples;

  // Convert to floating point
  k4a_imu_sample_t mean_float;
  mean.to_float(mean_float);
  // Use most timestamp of most recent sample
  mean_float.acc_timestamp_usec = samples.back().acc_timestamp_usec;
  mean_float.gyro_timestamp_usec = samples.back().gyro_timestamp_usec;

  return mean_float;
}

void K4AROS2Device::imuPublisherThread()
{
  rclcpp::Rate loop_rate(300);

  k4a_result_t result;
  k4a_imu_sample_t sample;

  // For IMU throttling
  unsigned int count = 0;
  unsigned int target_count = IMU_MAX_RATE / this->get_parameter("imu_rate_target").as_int();
  std::vector<k4a_imu_sample_t> accumulated_samples;
  accumulated_samples.reserve(target_count);
  bool throttling = target_count > 1;

  RCLCPP_DEBUG(this->get_logger(), "Starting IMU read loop.");

  //while (running_ && ros::ok() && !ros::isShuttingDown())
  while (running_ && rclcpp::ok())
  {
    if (k4a_device_)
    {
      // IMU messages are delivered in batches at 300 Hz. Drain the queue of IMU messages by
      // constantly reading until we get a timeout
      bool read = false;
      do
      {
        read = k4a_device_.get_imu_sample(&sample, std::chrono::milliseconds(0));

        if (read)
        {

          if (throttling)
          {
            accumulated_samples.push_back(sample);
            count++;
          }

          if (count % target_count == 0)
          {
            //ImuPtr imu_msg(new Imu);
            std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>();


            if (throttling)
            {
              k4a_imu_sample_t mean_sample_float = computeMeanIMUSample(accumulated_samples);
              result = getImuFrame(mean_sample_float, imu_msg);
              accumulated_samples.clear();
              count = 0;
            }
            else
            {
              result = getImuFrame(sample, imu_msg);
            }

            if (result != K4A_RESULT_SUCCEEDED)
            {
              RCLCPP_FATAL(this->get_logger(), "Failed to get IMU frame, shutting down");
              rclcpp::shutdown();
            }


            imu_orientation_publisher_->publish(*imu_msg);
          }
        }

      } while (read);
    }
    else if (k4a_playback_handle_)
    {
      // publish imu messages as long as the imu timestamp is less than the last capture timestamp to catch up to the
      // cameras compare signed with unsigned shouldn't cause a problem because timestamps should always be positive
      while (last_imu_time_usec_ <= last_capture_time_usec_ && !imu_stream_end_of_file_)
      {
        std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
        if (!k4a_playback_handle_.get_next_imu_sample(&sample))
        {
          imu_stream_end_of_file_ = true;
        }
        else
        {
          if (throttling)
          {
            accumulated_samples.push_back(sample);
            count++;
          }

          if (count % target_count == 0)
          {
            std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

            if (throttling)
            {
              k4a_imu_sample_t mean_sample_float = computeMeanIMUSample(accumulated_samples);
              result = getImuFrame(mean_sample_float, imu_msg);
              accumulated_samples.clear();
              count = 0;
            }
            else
            {
              result = getImuFrame(sample, imu_msg);
            }

            if (result != K4A_RESULT_SUCCEEDED)
            {
              RCLCPP_FATAL(this->get_logger(), "Failed to get IMU frame, shutting down");
              rclcpp::shutdown();
            }

            imu_orientation_publisher_->publish(*imu_msg);
            last_imu_time_usec_ = sample.acc_timestamp_usec;
          }
        }
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Frame grab cycle complete, sleeping...");
    loop_rate.sleep();
  }
}

std::chrono::microseconds K4AROS2Device::getCaptureTimestamp(const k4a::capture& capture)
{
  // Captures don't actually have timestamps, images do, so we have to look at all the images
  // associated with the capture.  We just return the first one we get back.
  //
  // We check the IR capture instead of the depth capture because if the depth camera is started
  // in passive IR mode, it only has an IR image (i.e. no depth image), but there is no mode
  // where a capture will have a depth image but not an IR image.
  //
  const auto irImage = capture.get_ir_image();
  if (irImage != nullptr)
  {
    return irImage.get_device_timestamp();
  }

  const auto colorImage = capture.get_color_image();
  if (colorImage != nullptr)
  {
    return colorImage.get_device_timestamp();
  }

  return std::chrono::microseconds::zero();
}

// Converts a k4a *device* timestamp to a ros::Time object
rclcpp::Time K4AROS2Device::timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
  // This will give INCORRECT timestamps until the first image.
  if (device_to_realtime_offset_.count() == 0)
  {
    initializeTimestampOffset(k4a_timestamp_us);
  }

  std::chrono::nanoseconds timestamp_in_realtime = k4a_timestamp_us + device_to_realtime_offset_;
  // Set as ROS_TIME clock
  rclcpp::Time ros_time(timestamp_in_realtime.count(), RCL_ROS_TIME);

  return ros_time;
}

// Converts a k4a_imu_sample_t timestamp to a ros::Time object
rclcpp::Time K4AROS2Device::timestampToROS(const uint64_t& k4a_timestamp_us)
{
  return timestampToROS(std::chrono::microseconds(k4a_timestamp_us));
}

void K4AROS2Device::initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us)
{
  // We have no better guess than "now".
  std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();

  device_to_realtime_offset_ = realtime_clock - k4a_device_timestamp_us;

  RCLCPP_WARN_STREAM(this->get_logger(), "Initializing the device to realtime offset based on wall clock: "
                  << device_to_realtime_offset_.count() << " ns");
}

void K4AROS2Device::updateTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us,
                                         const std::chrono::nanoseconds& k4a_system_timestamp_ns)
{
  // System timestamp is on monotonic system clock.
  // Device time is on AKDK hardware clock.
  // We want to continuously estimate diff between realtime and AKDK hardware clock as low-pass offset.
  // This consists of two parts: device to monotonic, and monotonic to realtime.

  // First figure out realtime to monotonic offset. This will change to keep updating it.
  std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();
  std::chrono::nanoseconds monotonic_clock = std::chrono::steady_clock::now().time_since_epoch();

  std::chrono::nanoseconds monotonic_to_realtime = realtime_clock - monotonic_clock;

  // Next figure out the other part (combined).
  std::chrono::nanoseconds device_to_realtime =
      k4a_system_timestamp_ns - k4a_device_timestamp_us + monotonic_to_realtime;
  // If we're over a second off, just snap into place.
  if (device_to_realtime_offset_.count() == 0 ||
      std::abs((device_to_realtime_offset_ - device_to_realtime).count()) > 1e7)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Initializing or re-initializing the device to realtime offset: "
      << device_to_realtime.count() << " ns");

    device_to_realtime_offset_ = device_to_realtime;
  }
  else
  {
    // Low-pass filter!
    constexpr double alpha = 0.10;
    device_to_realtime_offset_ = device_to_realtime_offset_ +
                                 std::chrono::nanoseconds(static_cast<int64_t>(
                                     std::floor(alpha * (device_to_realtime - device_to_realtime_offset_).count())));
  }
}




void K4AROS2Device::printTimestampDebugMessage(const std::string& name, const rclcpp::Time& timestamp)
{

  rclcpp::Time now(this->now(), RCL_ROS_TIME);
  rclcpp::Duration lag = now - timestamp;

  auto it = map_min_max_.find(name);
  if (it == map_min_max_.end())
  {
    map_min_max_.insert(std::make_pair(name, std::make_pair(lag, lag)));
    it = map_min_max_.find(name);
  }
  else
  {
    auto& min_lag = it->second.first;
    auto& max_lag = it->second.second;
    if (lag < min_lag)
    {
      min_lag = lag;
    }
    if (lag > max_lag)
    {
      max_lag = lag;
    }
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), name << " timestamp lags node Time::now() by\n"
                        << std::setw(23) << lag.seconds() * 1000.0 << " ms. "
                        << "The lag ranges from " << it->second.first.seconds() * 1000.0 << "ms"
                        << " to " << it->second.second.seconds() * 1000.0 << "ms.");
}
