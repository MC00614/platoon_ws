from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from collections import deque

class VelocityControl(Node):
    def __init__(self, truck_id):
        self.truck_id = truck_id
        
        node_name = f'truck{self.truck_id}_velocity_controller'
        super().__init__(node_name)
        
        topic_name = f'/truck{self.truck_id}/velocity_control'
        self.velocity_control_publisher_ = self.create_publisher(Float32, topic_name, 10)

        topic_name = f'/truck{self.truck_id}/velocity'
        self.ego_velocity_sub = self.create_subscription(
            Float32,
            topic_name,
            self.ego_velocity_callback,
            qos_profile_sensor_data)

        if self.truck_id == 0:
            topic_name = f'/platoon/target_velocity'
        else:
            topic_name = f'/platoon/truck{self.truck_id}/optimal_velocity'
        self.target_velocity_sub = self.create_subscription(
            Float32,
            topic_name,
            self.target_velocity_callback,
            qos_profile_sensor_data)

        # Initialize Variable
        self.target_velocity = 0.001
        self.velocity = 0.0
        self.max_throttle = 1.0
        self.min_throttle = 0.05
        self.publish_interval = 0.1

        # PID Controller
        self.k_p = 0.15
        self.k_i = 0.25
        self.k_d = 0.07
        self.integral_scope = 10 # Larger than 3
        self.error_list = deque(maxlen=self.integral_scope)
        
        # Release Hand Brake
        self.publish_velocity_control(1.0)
        
        self.publish_timer = self.create_timer(self.publish_interval, self.publish_timer_callback)

    def ego_velocity_callback(self, msg):
        self.velocity = msg.data
    
    def target_velocity_callback(self, msg):
        self.target_velocity = msg.data
        
    def publish_timer_callback(self):
        velocity_error = self.velocity - self.target_velocity
        self.error_list.append(velocity_error)
        
        if len(self.error_list) < 3:
            return

        print(f'velocity_error = {velocity_error}')

        error_sign = 1.0
        if velocity_error > 0:
            error_sign = - 1.0

        if self.target_velocity < 0.01:
            throttle_control = -1.0
        elif velocity_error == 0:
            throttle_control = 0.0
        else:
            control_p = self.k_p * velocity_error
            control_i = self.k_i * sum(self.error_list)
            control_d = self.k_d * ((self.error_list[-1] - self.error_list[-2])) / self.publish_interval
            # print(f'control_p = {control_p}')
            # print(f'control_i = {control_i}')
            # print(f'control_d = {control_d}')
            throttle_control = min(self.max_throttle, abs(control_p + control_i + control_d)) * error_sign
        print(f'throttle_control = {throttle_control}')

        # throttle_control = min(self.min_throttle, )
        if (0 < throttle_control < 0.05):
            throttle_control = 0.0
        elif (self.max_throttle < throttle_control):
            throttle_control = self.max_throttle
            
        self.publish_velocity_control(throttle_control)
            
    def publish_velocity_control(self, throttle_control):
        velocity_control_msg = Float32()
        velocity_control_msg.data = throttle_control
        self.velocity_control_publisher_.publish(velocity_control_msg)

    def __del__(self):
        self.publish_velocity_control(0.0)
        



