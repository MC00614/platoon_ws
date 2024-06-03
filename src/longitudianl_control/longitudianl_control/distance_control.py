from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

import control as ct
import numpy as np

class DistanceControl(Node):
    def __init__(self, truck_id):
        self.truck_id = truck_id
        node_name = f'truck{self.truck_id}_distance_controller'
        super().__init__(node_name)

        topic_name = f'/platoon/truck{self.truck_id}/optimal_velocity'
        self.velocity_control_publisher_ = self.create_publisher(Float32, topic_name, 10)

        self.target_velocity_sub = self.create_subscription(
            Float32,
            '/platoon/target_velocity',
            self.target_velocity_callback,
            qos_profile_sensor_data)

        self.target_velocity_sub = self.create_subscription(
            Float32,
            '/platoon/target_distance',
            self.target_distance_callback,
            qos_profile_sensor_data)

        topic_name = f'/truck{self.truck_id}/velocity'
        self.ego_velocity_sub = self.create_subscription(
            Float32,
            topic_name,
            self.ego_velocity_callback,
            qos_profile_sensor_data)
        
        topic_name = f'/platoon/truck{self.truck_id}/front_truck_pose'
        self.front_truck_pose_sub = self.create_subscription(
            Pose,
            topic_name,
            self.front_truck_pose_callback,
            qos_profile_sensor_data)
        
        # Initialize Variable
        self.target_velocity = 0.0
        self.target_distance = 20.0
        self.velocity = 0.0
        self.k_v = 1.0
        
        # Release Hand Brake
        # self.publish_optimal_velocity(1.0)
        # Changed in virtual_ws

        # LQR Controller
        dt = 1
        A = np.array([[1]])
        B = np.array([[dt]])
        Q = np.array([[0.2]])
        R = np.array([[0.1]])
        self.K, S, E = ct.lqr(A, B, Q, R)
        print(self.K)

        self.state = np.array([self.target_distance])
        # np.transpose(self.state)

    def target_velocity_callback(self, msg):
        self.target_velocity = msg.data

    def target_distance_callback(self, msg):
        self.target_distance = msg.data

    def ego_velocity_callback(self, msg):
        self.velocity = msg.data
    
    def front_truck_pose_callback(self, msg):
        front_truck_x = msg.position.x
        front_truck_y = msg.position.y

        x_weight = 1.0
        y_weight = 0.5
        truck_distance = (x_weight * (front_truck_x**2) + y_weight * (front_truck_y**2))**0.5

        self.state[0] = truck_distance - self.target_distance

        # print(f'distance = {self.state[0]}')

        optimal_velocity = self.k_v * np.dot(self.K, self.state)[0]
        print(f'optimal_velocity = {optimal_velocity}')
        if (0 < optimal_velocity < 0.05):
            optimal_velocity = 0.0

        self.publish_optimal_velocity(optimal_velocity)
            
    def publish_optimal_velocity(self, optimal_velocity):
        velocity_control_msg = Float32()
        velocity_control_msg.data = optimal_velocity
        self.velocity_control_publisher_.publish(velocity_control_msg)

    def __del__(self):
        self.publish_optimal_velocity(0.0)
        



