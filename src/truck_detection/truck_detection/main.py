#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import numpy as np
import math

class TruckDetection(Node):
    def __init__(self):
        super().__init__('truck_detection')
        
        self.front_truck_pub = self.create_publisher(Pose, '/platooning/mc_truck1/front_truck', 10)

        self.front_lidar_sub = self.create_subscription(
            PointCloud2,
            '/mc_truck1/front_lidar',
            self.front_lidar_callback,
            qos_profile_sensor_data)
        self.front_lidar_sub
        self.point_cloud_list = []

    def front_lidar_callback(self, msg):
        self.point_cloud_list = []
        
        offset_x = next(field.offset for field in msg.fields if field.name == 'x')
        offset_y = next(field.offset for field in msg.fields if field.name == 'y')
        # offset_z = next(field.offset for field in msg.fields if field.name == 'z')

        point_step = msg.point_step
        num_points = msg.width
        data = msg.data
        point_cloud = []
        for i in range(num_points):
            index = i * point_step
            x = np.frombuffer(data[index + offset_x:index + offset_x + 4], dtype=np.float32)[0]
            y = np.frombuffer(data[index + offset_y:index + offset_y + 4], dtype=np.float32)[0]
            point_cloud.append([x, y])

            # z = np.frombuffer(data[index + offset_z:index + offset_z + 4], dtype=np.float32)[0]
            # point_cloud.append([x, y, z])

        self.point_cloud_list.extend(point_cloud)

        min_distance_2 = math.inf
        min_distance_idx = -1
        distance_2_list = []
        for i in range(len(self.point_cloud_list)):
            x = self.point_cloud_list[i][0]
            y = self.point_cloud_list[i][1]
            distance_2 = x*x + y*y
            if distance_2 < min_distance_2:
                min_distance_idx = i
            distance_2_list.append(distance_2)
        print(distance_2_list)
        
        # L = [min_distance_idx]
        # while L:
        #     cur_idx = L.pop()
        #     if 0 <= cur_idx:
                
        
                
        
def main(args=None):
    rclpy.init(args=args)

    lane_detector = TruckDetection()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
