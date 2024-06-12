#!/usr/bin/env python3

from .lane_follower import LaneFollower


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
import argparse

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import math

import cv2

class LateralControl(Node):
    def __init__(self, truck_id):
        self.truck_id = truck_id
        node_name = f'truck{self.truck_id}_lateral_control'
        super().__init__(node_name)
        
        self.lane_follower = LaneFollower(width=640, height=480, max_steer=30.0, normal_throttle=1.0, k_o=0.4, k_c=0.6)

        topic_name = f'/platoon/truck{self.truck_id}/path'
        self.path_sub = self.create_subscription(
            Path,
            topic_name,
            self.path_callback,
            qos_profile_sensor_data)
        self.path_sub

        topic_name = f'/truck{self.truck_id}/steer_control'
        self.steering_control_publisher_ = self.create_publisher(Float32, topic_name, 10)

    def path_callback(self, msg):
        middle_points = []
        for pose_stamped in msg.poses:
            middle_point = (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            middle_points.append(middle_point)

        self.lane_follower.calculate_control(middle_points)
        _, steering = self.lane_follower.get_control()
        steering = - steering
        self.publish_steering_control(steering)

    def publish_steering_control(self, steering_control):
        steering_control_msg = Float32()
        steering_control_msg.data = steering_control
        self.steering_control_publisher_.publish(steering_control_msg)
            
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Lateral Control Node')
    parser.add_argument('--truck_id', type=int, help='Truck ID')
    parsed_args, _ = parser.parse_known_args()
    lateral_control = LateralControl(truck_id=parsed_args.truck_id)
    rclpy.spin(lateral_control)
    lateral_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
