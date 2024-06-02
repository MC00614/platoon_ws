from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

import control as ct
import numpy as np

class DistanceControl(Node):

    def __init__(self):
        super().__init__('distance_control')
        self.velocity_control_publisher_ = self.create_publisher(Float32, '/mc_truck1/velocity_control', 10)

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

        self.ego_velocity_sub = self.create_subscription(
            Float32,
            '/mc_truck1/velocity',
            self.ego_velocity_callback,
            qos_profile_sensor_data)

        self.front_truck_velocity_sub = self.create_subscription(
            Float32,
            '/mc_truck0/velocity',
            self.front_truck_velocity_callback,
            qos_profile_sensor_data)

        self.front_truck_pose_sub = self.create_subscription(
            Pose,
            '/platoon/mc_truck1/front_truck',
            self.front_truck_pose_callback,
            qos_profile_sensor_data)
        
        # Initialize Variable
        self.target_velocity = 0.0
        self.target_distance = 20.0
        self.velocity = 0.0
        self.front_truck_velocity = 0.0
        
        # Release Hand Brake
        self.publish_velocity_control(1.0)

        # LQR Controller
        dt = 1
        A = np.array([[1, dt], [0, 1]])
        B = np.array([[0.5*(dt**2)], [dt]])
        Q = np.array([[0.1, 0], [0, 0.2]])
        R = np.array([0.1])
        self.K, S, E = ct.lqr(A, B, Q, R)
        print(self.K)

        self.state = np.array([0.0, 0.0])
        # np.transpose(self.state)

    def target_velocity_callback(self, msg):
        self.target_velocity = msg.data

    def target_distance_callback(self, msg):
        self.target_distance = msg.data

    def ego_velocity_callback(self, msg):
        self.velocity = msg.data

    def front_truck_velocity_callback(self, msg):
        self.front_truck_velocity = msg.data
    
    def front_truck_pose_callback(self, msg):
        front_truck_x = msg.position.x
        front_truck_y = msg.position.y

        max_accel = 0.6

        x_weight = 1.0
        y_weight = 0.5
        truck_distance = (x_weight * (front_truck_x**2) + y_weight * (front_truck_y**2))**0.5

        self.state[0] = truck_distance - self.target_distance
        self.state[1] = self.velocity - self.target_velocity
        # self.state[1] = self.velocity - self.front_truck_velocity
        print(f'distance = {self.state[0]}')
        print(f'velocity = {self.state[1]}')

        throttle_control = np.dot(self.K, self.state)[0] / 100.0
        print(f'throttle_control = {throttle_control}')
        if (0 < throttle_control < 0.05):
            throttle_control = 0.0
        elif (max_accel < throttle_control):
            throttle_control = max_accel
            
        self.publish_velocity_control(throttle_control)
            
    def publish_velocity_control(self, throttle_control):
        velocity_control_msg = Float32()
        velocity_control_msg.data = throttle_control
        self.velocity_control_publisher_.publish(velocity_control_msg)

    def __del__(self):
        self.publish_velocity_control(0.0)
        



