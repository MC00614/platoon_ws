from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

class DistanceControl(Node):

    def __init__(self):
        super().__init__('distance_control')
        self.velocity_control_publisher_ = self.create_publisher(Float32, '/mc_truck1/velocity_control', 10)

        self.front_truck_sub = self.create_subscription(
            Pose,
            '/platoon/mc_truck1/front_truck',
            self.front_truck_callback,
            qos_profile_sensor_data)
    
    def front_truck_callback(self, msg):
        front_truck_x = msg.position.x
        front_truck_y = msg.position.y

        # brake_distance = 15
        # throttle_distance = 
        
        brake_distance_2 = 15 ** 2
        throttle_distance_2 = 20 ** 2
        base_accel = 0.6
        
        if ((front_truck_x**2 + front_truck_y**2) < brake_distance_2):
            velocity_control = -base_accel
        elif ((front_truck_x**2 + front_truck_y**2) > throttle_distance_2):
            velocity_control = base_accel
        else:
            velocity_control = 0.0
            
        self.publish_velocity_control(velocity_control)
            
    def publish_velocity_control(self, velocity_control):
        velocity_control_msg = Float32()
        velocity_control_msg.data = velocity_control
        self.velocity_control_publisher_.publish(velocity_control_msg)

    def __del__(self):
        self.publish_velocity_control(0.0)
        



