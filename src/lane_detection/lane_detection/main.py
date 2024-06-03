#!/usr/bin/env python3

from .class_image_processor import ImageProcessor

from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import math

import cv2

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')
        
        self.image_processor = ImageProcessor()

        self.image_sub0 = self.create_subscription(
            Image,
            '/truck0/front_camera',
            self.img_callback0,
            qos_profile_sensor_data)
        self.image_sub0
        
        self.image_sub1 = self.create_subscription(
            Image,
            '/mc_truck1/front_camera',
            self.img_callback1,
            qos_profile_sensor_data)
        self.image_sub1 
        
        self.image_sub2 = self.create_subscription(
            Image,
            '/mc_truck2/front_camera',
            self.img_callback2,
            qos_profile_sensor_data)
        self.image_sub2 

        self.path_publisher0 = self.create_publisher(
            Path, 
            'truck0/path',
            10)

        self.path_publisher1 = self.create_publisher(
            Path, 
            'truck1/path',
            10)

        self.path_publisher2 = self.create_publisher(
            Path, 
            'truck2/path',
            10)
        
        self.bridge = CvBridge()
        self.image0 = None
        self.image1 = None
        self.image2 = None

    def img_callback0(self, msg):
        # print(0)
        try:
            self.image0 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            middle_points = self.image_processor.frame_processor(self.image0)
            height, width, channels = self.image0.shape
            relative_points = self.image_processor.calculate_relative_path(middle_points, height, width)
            print(f"Middle Points : {relative_points}")
            # print(f"Height: {height}, Width: {width}, Channels: {channels}")
            # PUBLISH HERE
            self.publish_path(relative_points)

            cv2.imshow("self.image0", self.image0)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e0)

    def img_callback1(self, msg):
        # print(1)
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            middle_points = self.image_processor.frame_processor(self.image1)
            height, width, channels = self.image1.shape
            relative_points = self.image_processor.calculate_relative_path(middle_points, height, width)
            cv2.imshow("self.image1", self.image1)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e1)

    def img_callback2(self, msg):
        print(2)
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            middle_points = self.image_processor.frame_processor(self.image2)
            height, width, channels = self.image2.shape
            relative_points = self.image_processor.calculate_relative_path(middle_points, height, width)
            cv2.imshow("self.image2", self.image2)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e2)
    
    def publish_path(self, relative_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in relative_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = self.yaw_to_quaternion(point[2])
            path_msg.poses.append(pose_stamped)

        self.path_publisher0.publish(path_msg)
    
    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.w = math.cos(yaw / 2.0)
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        return quaternion
            
def main(args=None):
    rclpy.init(args=args)

    lane_detector = LaneDetection()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
