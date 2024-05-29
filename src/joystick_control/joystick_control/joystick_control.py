from .gamepads import ShanWanGamepad

from rclpy.node import Node
from std_msgs.msg import Float32

class JoystickControl(Node):

    def __init__(self):
        super().__init__('joystick_control')
        self.shanwan_gamepad = ShanWanGamepad()
        self.steer_control_publisher_ = self.create_publisher(Float32, '/mc_truck0/steer_control', 10)
        self.velocity_control_publisher_ = self.create_publisher(Float32, '/mc_truck0/velocity_control', 10)

        while True:
            gamepad_input = self.shanwan_gamepad.read_data()
            if (abs(gamepad_input.analog_stick_left.x) < 0.1): 
                gamepad_input.analog_stick_left.x = 0.0
            if (abs(gamepad_input.analog_stick_right.y) < 0.1):
                gamepad_input.analog_stick_right.y = 0.0
            steer_control = - gamepad_input.analog_stick_left.x * 30.0
            velocity_control = gamepad_input.analog_stick_right.y * 0.5
            self.publish_steer_control(steer_control)
            self.publish_velocity_control(velocity_control)
        
    def publish_steer_control(self, steer_control):
        steer_control_msg = Float32()
        steer_control_msg.data = steer_control
        self.steer_control_publisher_.publish(steer_control_msg)
        
    def publish_velocity_control(self, velocity_control):
        velocity_control_msg = Float32()
        velocity_control_msg.data = velocity_control
        self.velocity_control_publisher_.publish(velocity_control_msg)

    def __del__(self):
        self.publish_steer_control(0.0)
        self.publish_velocity_control(0.0)
        



