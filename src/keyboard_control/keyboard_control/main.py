from .keyboard_control import KeyboardControl

import rclpy

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()