#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from cv_bridge import CvBridge

import numpy as np
g_node = None
cv_bridge = None

def imu_callback(msg):
    global g_node

    g_node.get_logger().info(
        'IMU message: x: {:0.2f}  y: {:0.2f}  z: {:0.2f}'.format(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))


def rgb_raw_callback(msg):
    global g_node
    global cv_bridge

    g_node.get_logger().info('RGB raw image message of type {} with width: {}  height: {}'.format(type(msg), msg.width, msg.height))

    img = cv_bridge.imgmsg_to_cv2(msg)
    cv2.imshow("RGB Raw Image", img)
    cv2.waitKey(1)


def depth_rgb_callback(msg):
    global g_node
    global cv_bridge

    g_node.get_logger().info('Depth->RGB raw image message of type {} with width: {}  height: {}'.format(type(msg), msg.width, msg.height))

    img = cv_bridge.imgmsg_to_cv2(msg)
    cv2.imshow("Depth->RGB Raw Image", img)
    cv2.waitKey(1)

def rgb_rect_callback(msg):
    global g_node
    global cv_bridge

    g_node.get_logger().info('RGB rect image message of type {} with width: {}  height: {}'.format(type(msg), msg.width, msg.height))

    img = cv_bridge.imgmsg_to_cv2(msg)
    cv2.imshow("RGB Rect Image", img)
    cv2.waitKey(1)


def rgb_info_callback(msg):
    global g_node

    g_node.get_logger().info('RGB camera info message: D: {}  K: {}'.format(msg.d, msg.k))


def depth_raw_callback(msg):
    global g_node

    g_node.get_logger().info('Depth raw image message of type {} with width: {}  height: {}'.format(type(msg), msg.width, msg.height))

    dimg = cv_bridge.imgmsg_to_cv2(msg)
    cv2.imshow("Depth Raw Image", dimg)
    cv2.waitKey(1)


def depth_rect_callback(msg):
    global g_node

    g_node.get_logger().info('Depth rect image message of type {} with width: {}  height: {}'.format(type(msg), msg.width, msg.height))

    dimg = cv_bridge.imgmsg_to_cv2(msg)
    cv2.imshow("Depth Rect Image", dimg)
    cv2.waitKey(1)

def depth_info_callback(msg):
    global g_node

    g_node.get_logger().info('Depth camera info message received')

def main(args=None):
    global g_node
    global cv_bridge

    rclpy.init(args=args)

    g_node = rclpy.create_node('test_azure_kinect_subscriber')
    g_node.get_logger().info("Starting %s" % g_node.get_name())

    cv_bridge = CvBridge()

    #subscription_imu = g_node.create_subscription(Imu, '/k4a/imu', imu_callback, 10)
    #g_node.get_logger().info("Subscribed to %s" % subscription_imu.topic_name)

    #subscription_rgb_rect = g_node.create_subscription(Image, '/k4a/rgb_to_depth/image_rect', rgb_rect_callback, 10)
    #g_node.get_logger().info("Subscribed to %s" % subscription_rgb_rect.topic_name)

    #subscription_rgb_raw = g_node.create_subscription(Image, '/k4a/rgb_to_depth/image_raw', rgb_raw_callback, 10)
    #g_node.get_logger().info("Subscribed to %s" % subscription_rgb_raw.topic_name)

    subscription_rgb_raw = g_node.create_subscription(Image, '/k4a/rgb/image_raw', rgb_raw_callback, 10)
    g_node.get_logger().info("Subscribed to %s" % subscription_rgb_raw.topic_name)


    #subscription_depth_raw = g_node.create_subscription(Image, '/k4a/depth/image_raw', depth_raw_callback, 10)
    #g_node.get_logger().info("Subscribed to %s" % subscription_depth_raw.topic_name)

    #subscription_depth_rect = g_node.create_subscription(Image, '/k4a/depth/image_rect', depth_rect_callback, 10)
    #g_node.get_logger().info("Subscribed to %s" % subscription_depth_rect.topic_name)

    subscription_depth_to_rgb = g_node.create_subscription(Image, '/k4a/depth_to_rgb/image_raw', depth_rgb_callback, 10)
    g_node.get_logger().info("Subscribed to %s" % subscription_depth_to_rgb.topic_name)

    subscription_rgb_info = g_node.create_subscription(CameraInfo, '/k4a/rgb/camera_info', rgb_info_callback, 10)
    g_node.get_logger().info("Subscribed to %s" % subscription_rgb_info.topic_name)

    #subscription_depth_info = g_node.create_subscription(CameraInfo, '/k4a/depth/camera_info', depth_info_callback, 10)
    #g_node.get_logger().info("Subscribed to %s" % subscription_depth_info.topic_name)


    while rclpy.ok():
        rclpy.spin_once(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
