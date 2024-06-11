import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import argparse

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import math

from scipy import stats
from collections import defaultdict

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

        point_step = msg.point_step
        num_points = msg.width
        data = msg.data
        point_cloud = []
        for i in range(num_points):
            index = i * point_step
            x = np.frombuffer(data[index + offset_x:index + offset_x + 4], dtype=np.float32)[0]
            if x < 0:
                continue
            y = np.frombuffer(data[index + offset_y:index + offset_y + 4], dtype=np.float32)[0]
            point_cloud.append([x, y])

        self.point_cloud_list.extend(point_cloud)
        analyzer = PointAnalyzer(self.point_cloud_list)
        new_points = analyzer.analyze_points()
        if not new_points:
            #print("AAA")
            m = len(self.point_cloud_list)
            if m == 0:
                average_x = 20.0
                average_y = 0.0
            else:
                sum_x = sum(point[0] for point in self.point_cloud_list)
                average_x = sum_x / m + 5.0 
                average_y = 0.0
        else:
            #print("BBB")
            m = len(new_points)
            if m == 0:
                average_x = 20.0
                average_y = 0.0
            else:
                sum_x = sum(point[0] for point in new_points)
                sum_y = sum(point[1] for point in new_points)  
                average_x = sum_x / m + 5.0
                average_y = sum_y / m  

        self.publish_front_truck(average_x, average_y)

    def publish_front_truck(self, x, y):
        front_truck_msg = Pose()
        front_truck_msg.position.x = x
        front_truck_msg.position.y = y
        self.front_truck_pose_pub.publish(front_truck_msg)

class PointAnalyzer:
    def __init__(self, point_cloud_list):
        self.point_cloud_list = point_cloud_list

    def find_valid_segments(self):
        angle_tolerance = 0.03
        segments = []
        self.point_cloud_list = [point for point in self.point_cloud_list if not (point[0] == 0.0 and point[1] == 0.0)]
        print("Total points", len(self.point_cloud_list))
        if len(self.point_cloud_list) > 1:
            sorted_seg = sorted(self.point_cloud_list, key=lambda point: point[1])
            #print(sorted_seg)
            if len(sorted_seg) >= 20:
                sloupes = []
                for i in range(int(len(sorted_seg) / 3)):  
                    slope = (sorted_seg[i+3][0] - sorted_seg[i][0]) / (sorted_seg[i+3][1] - sorted_seg[i][1] + 0.01)
                    sloupes.append(slope)
                #print("SLOPES", sloupes[0])

                slope_groups = defaultdict(list)
                for slope in sloupes:
                    found_group = False
                    for key in slope_groups.keys():
                        if abs(slope - key) <= angle_tolerance:
                            slope_groups[key].append(slope)
                            found_group = True
                            break
                    if not found_group:
                        slope_groups[slope].append(slope)
                
                most_frequent_group = max(slope_groups.values(), key=len)
                most_frequent_slope = most_frequent_group[0]
                #print("PERVIY", most_frequent_slope)
                #print("Degree angle1 = ", math.degrees(math.atan(most_frequent_slope)))
            else:
                most_frequent_slope = (sorted_seg[3][0] - sorted_seg[0][0]) / (sorted_seg[3][1] - sorted_seg[0][1] + 0.01)
                #print("VTOROI", most_frequent_slope)

            for i in range(1, len(sorted_seg)-3):
                angle = (sorted_seg[i+3][0] - sorted_seg[i][0]) / (sorted_seg[i+3][1] - sorted_seg[i][1] + 0.01)
                if abs(most_frequent_slope - angle) < angle_tolerance:                
                    segments.append(sorted_seg[i])
            segments = sorted(segments, key=lambda point: point[1])
            print("SEGMENTS", len(segments))
            return segments
        else:
            print("EMPTY")
            return []

    def analyze_points(self):
        sorted_points = self.find_valid_segments()
        if  int(len(sorted_points)) < 5:
            print("No valid segments found.")
            return []

        sorted_points = sorted(sorted_points, key=lambda point: point[1])
        #print("Total lane points", len(sorted_points))
        sorted_x = [point[0] for point in sorted_points]
        sorted_y = [point[1] for point in sorted_points]
        if len(sorted_x) < 2 or len(sorted_y) < 2:
            print("Not enough points for linear regression.")
            return []


        slope, intercept, r_value, p_value, std_err = stats.linregress(sorted_y, sorted_x)
        r_squared = r_value**2
        line_angle = math.degrees(math.atan(slope))
        print("ANGLE = ", line_angle)
        threshold = 0.97

        if r_squared > threshold and (-35 <= line_angle <= 3):
            print("Points form a line with R-squared:", r_squared)
            return sorted_points 
        else:
            print("Points do not form a sufficient line:", r_squared)    
            return []

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Truck Detection Node')
    parser.add_argument('--truck_id', type=int, help='Truck ID')
    parsed_args, _ = parser.parse_known_args()
    truck_detection = TruckDetection(truck_id=parsed_args.truck_id)

    try:
        rclpy.spin(truck_detection)
    except KeyboardInterrupt:
        truck_detection.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        if rclpy.ok():
            truck_detection.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
