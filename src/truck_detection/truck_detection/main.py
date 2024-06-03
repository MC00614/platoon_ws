#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import argparse

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import numpy as np
import math

class TruckDetection(Node):
    def __init__(self, truck_id):
        self.truck_id = truck_id
        node_name = f'truck{self.truck_id}_truck_detection'
        super().__init__(node_name)
        
        topic_name = f'/platoon/truck{self.truck_id}/front_truck_pose'
        self.front_truck_pose_pub = self.create_publisher(Pose, topic_name, 10)

        topic_name = f'/truck{self.truck_id}/front_lidar'
        self.front_lidar_sub = self.create_subscription(
            PointCloud2,
            topic_name,
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
        
        n = len(self.point_cloud_list)
        min_distance_2 = math.inf
        min_distance_idx = -1
        distance_2_list = []
        for i in range(n):
            x = self.point_cloud_list[i][0]
            y = self.point_cloud_list[i][1]
            distance_2 = x*x + y*y
            if distance_2 < min_distance_2:
                min_distance_idx = i
                min_distance_2 = distance_2
            distance_2_list.append(distance_2)

        average_x = 0
        average_y = 0
        index_count = 0
        
        if min_distance_idx == -1:
            average_x = 30.0
            average_y = 0.0
            self.publish_front_truck(average_x, average_y)
            return

        distance_threshold_2 = 25
        L = [min_distance_idx]
        V = [False for _ in range(n)]
        V[min_distance_idx] = True
        while L:
            current_index = L.pop()
            if (0 < current_index):
                next_index = current_index - 1
                if V[next_index] == False:
                    if (abs(distance_2_list[next_index] - distance_2_list[current_index]) < distance_threshold_2):
                        average_x += self.point_cloud_list[next_index][0]
                        average_y += self.point_cloud_list[next_index][1]
                        index_count += 1
                        V[next_index] = True
                        L.append(next_index)

            elif (current_index + 1 < n):
                next_index = current_index + 1
                if V[next_index] == False:
                    if (abs(distance_2_list[next_index] - distance_2_list[current_index]) < distance_threshold_2): 
                        average_x += self.point_cloud_list[next_index][0]
                        average_y += self.point_cloud_list[next_index][1]
                        index_count += 1
                        V[next_index] = True
                        L.append(next_index)
        
        if index_count != 0:
            average_x /= index_count
            average_y /= index_count
        else:
            average_x = 30.0
            average_y = 0.0

        self.publish_front_truck(average_x, average_y)
        print(f'average_x = {average_x}')
        print(f'average_y = {average_y}')
        print(f'distance = {(average_x**2 + average_y**2)**0.5}')

    def publish_front_truck(self, x, y):
        front_truck_msg = Pose()
        front_truck_msg.position.x = x
        front_truck_msg.position.y = y
        self.front_truck_pub.publish(front_truck_msg)
                
                
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Truck Detection Node')
    parser.add_argument('--truck_id', type=int, help='Truck ID')
    parsed_args, _ = parser.parse_known_args()
    truck_detection = TruckDetection(truck_id=parsed_args.truck_id)
    rclpy.spin(truck_detection)
    truck_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
