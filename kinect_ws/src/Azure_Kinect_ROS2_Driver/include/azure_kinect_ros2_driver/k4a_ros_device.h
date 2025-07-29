// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_DEVICE_H
#define K4A_ROS_DEVICE_H

// System headers
//
#include <atomic>
#include <mutex>
#include <thread>
#include <map>

// Library headers
//
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <k4a/k4a.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.h>
#include <k4arecord/playback.hpp>



// Project headers
//
#include "azure_kinect_ros2_driver/k4a_calibration_transform_data.h"
#include "azure_kinect_ros2_driver/k4a_ros_device_params.h"



class K4AROS2Device : public rclcpp::Node
{

  public:

    explicit K4AROS2Device();

    ~K4AROS2Device() override;

    k4a_result_t startCameras();
    k4a_result_t startImu();

    void stopCameras();
    void stopImu();

    // Get camera calibration information for the depth camera
    //void getDepthCameraInfo(std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info);
    //void getRgbCameraInfo(std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info);

    k4a_result_t getDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& depth_frame, bool rectified);

    k4a_result_t getPointCloud(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud);

    k4a_result_t getRgbPointCloudInRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud);
    k4a_result_t getRgbPointCloudInDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud);

    k4a_result_t getImuFrame(const k4a_imu_sample_t& capture, std::shared_ptr<sensor_msgs::msg::Imu> imu_frame);

    k4a_result_t getRbgFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& rgb_frame, bool rectified);
    k4a_result_t getJpegRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::CompressedImage> jpeg_image);

    k4a_result_t getIrFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& ir_image);

    void printTimestampDebugMessage(const std::string& name, const rclcpp::Time& timestamp);

  private:

    k4a_result_t renderBGRA32ToROS(std::shared_ptr<sensor_msgs::msg::Image>& rgb_frame, k4a::image& k4a_bgra_frame);
    k4a_result_t renderDepthToROS(std::shared_ptr<sensor_msgs::msg::Image>&  depth_image, k4a::image& k4a_depth_frame);
    k4a_result_t renderIrToROS(std::shared_ptr<sensor_msgs::msg::Image>& ir_image, k4a::image& k4a_ir_frame);

    k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud);
    k4a_result_t fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                     std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);

    void framePublisherThread();
    void imuPublisherThread();

    // Gets a timestap from one of the captures images
    std::chrono::microseconds getCaptureTimestamp(const k4a::capture& capture);

    // Converts a k4a_image_t timestamp to a ros::Time object
    rclcpp::Time timestampToROS(const std::chrono::microseconds& k4a_timestamp_us);

    // Converts a k4a_imu_sample_t timestamp to a ros::Time object
    rclcpp::Time timestampToROS(const uint64_t& k4a_timestamp_us);

    // Updates the timestamp offset (stored as start_time_) between the device time and ROS time.
    // This is a low-pass filtered update based on the system time from k4a, which represents the
    // time the message arrived at the USB bus.
    void updateTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us,
                               const std::chrono::nanoseconds& k4a_system_timestamp_ns);
    // Make an initial guess based on wall clock. The best we can do when no image timestamps are
    // available.
    void initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us);

    // When using IMU throttling, computes a mean measurement from a set of IMU samples
    k4a_imu_sample_t computeMeanIMUSample(const std::vector<k4a_imu_sample_t>& samples);

    // TODO: QoS Params
    rclcpp::QoS qos_;

    image_transport::CameraPublisher rgb_raw_publisher_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> rgb_cam_info_jpeg_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> rgb_jpeg_publisher_;

    image_transport::CameraPublisher depth_raw_publisher_;
    image_transport::CameraPublisher depth_rect_publisher_;
    image_transport::CameraPublisher rgb_rect_publisher_;
    image_transport::CameraPublisher ir_raw_publisher_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_orientation_publisher_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher_;


    std::shared_ptr<sensor_msgs::msg::CameraInfo> rgb_raw_camerainfo_msg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> rgb_jpeg_camerainfo_msg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> ir_raw_camerainfo_msg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> rgb_rect_camerainfo_msg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> depth_rect_camerainfo_msg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> depth_raw_camerainfo_msg_;

    // Are point clouds enabled
    bool process_cloud_;

    // K4A device
    k4a::device k4a_device_;
    std::unique_ptr<K4ACalibrationTransformData> calibration_data_;

    // K4A Recording
    k4a::playback k4a_playback_handle_;
    std::mutex k4a_playback_handle_mutex_;



    std::chrono::nanoseconds device_to_realtime_offset_{0};

    // Thread control
    volatile bool running_;

    // Last capture timestamp for synchronizing playback capture and imu thread
    std::atomic_int64_t last_capture_time_usec_;

    // Last imu timestamp for synchronizing playback capture and imu thread
    std::atomic_uint64_t last_imu_time_usec_;
    std::atomic_bool imu_stream_end_of_file_;

    // Threads
    std::thread frame_publisher_thread_;
    std::thread imu_publisher_thread_;

    std::map< std::string, std::pair<rclcpp::Duration, rclcpp::Duration> > map_min_max_;
};

#endif  // K4A_ROS_DEVICE_H
