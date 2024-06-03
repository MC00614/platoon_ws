#!/usr/bin/env python3

from .class_image_processor import ImageProcessor

from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import cv2

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')
        
        self.image_processor = ImageProcessor()

        self.image_sub = self.create_subscription(
            Image,
            '/mc_truck0/front_camera',
            self.img_callback,
            qos_profile_sensor_data)
        self.image_sub
        
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
        
        self.bridge = CvBridge()
        self.image = None
        self.image1 = None
        self.image2 = None

    def img_callback(self, msg):
        print(0)
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_processor.frame_processor(self.image)
            # PUBLISH HERE
            
            # cv2.imshow("self.image", self.image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def img_callback1(self, msg):
        print(1)
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("self.image1", self.image1)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def img_callback2(self, msg):
        print(2)
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("self.image2", self.image2)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            
def main(args=None):
    rclpy.init(args=args)

    lane_detector = LaneDetection()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
