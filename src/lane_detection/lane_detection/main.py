#!/usr/bin/env python3

from .class_image_processor import ImageProcessor

from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import argparse
import cv2

class LaneDetection(Node):
    def __init__(self, truck_id):
        self.truck_id = truck_id
        node_name = f'truck{self.truck_id}_lane_detection'
        super().__init__(node_name)
        
        self.image_processor = ImageProcessor()

        topic_name = f'/truck{self.truck_id}/front_camera'
        self.image_sub = self.create_subscription(
            Image,
            topic_name,
            self.img_callback,
            qos_profile_sensor_data)
        self.image_sub
        
        self.bridge = CvBridge()
        self.image = None

    def img_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            middle_points = self.image_processor.frame_processor(self.image)
            # PUBLISH HERE
            # self.publish_lane(middle_points)

            # cv2.imshow(f'front_image/{self.truck_id}', self.image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Lane Detection Node')
    parser.add_argument('--truck_id', type=int, help='Truck ID')
    parsed_args, _ = parser.parse_known_args()
    lane_detector = LaneDetection(truck_id=parsed_args.truck_id)
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
