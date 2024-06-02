from .velocity_control import VelocityControl

import rclpy

def main(args=None):
    rclpy.init(args=args)
    velocity_control = VelocityControl()
    rclpy.spin(velocity_control)
    velocity_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()