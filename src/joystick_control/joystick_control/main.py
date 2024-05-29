from .joystick_control import JoystickControl

import rclpy

def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    joystick_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()