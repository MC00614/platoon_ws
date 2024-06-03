from rclpy.node import Node
from std_msgs.msg import Float32

import pygame

class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.steer_control_publisher_ = self.create_publisher(Float32, '/truck0/steer_control', 10)
        self.velocity_control_publisher_ = self.create_publisher(Float32, '/truck0/velocity_control', 10)
        
        # Initialize pygame and set up the display
        pygame.init()
        self.screen = pygame.display.set_mode((100, 100))
        pygame.display.set_caption('Keyboard Control')

        self.steer_control = 0.0
        self.velocity_control = 0.0
        
        # Create a timer to call the keyboard_callback periodically
        self.timer = self.create_timer(0.1, self.keyboard_callback)

    def keyboard_callback(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                return

        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_a]:
            self.steer_control = max(self.steer_control - 1.0, -30.0)  # Left
        elif keys[pygame.K_d]:
            self.steer_control = min(self.steer_control + 1.0, 30.0)  # Right
        else:
            self.steer_control = 0.0
        
        if keys[pygame.K_w]:
            self.velocity_control = min(self.velocity_control + 1.0, 30.0)  # Forward
        elif keys[pygame.K_s]:
            self.velocity_control = max(self.velocity_control - 1.0, -30.0)  # Backward
        else:
            self.velocity_control = 0.0
        
        self.publish_steer_control(self.steer_control)
        self.publish_velocity_control(self.velocity_control)
        
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
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
