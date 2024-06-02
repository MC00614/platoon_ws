from .distance_control import DistanceControl

import rclpy

def main(args=None):
    rclpy.init(args=args)
    distance_control = DistanceControl()
    rclpy.spin(distance_control)
    distance_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()