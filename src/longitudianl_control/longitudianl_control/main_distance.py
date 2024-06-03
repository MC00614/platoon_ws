from .distance_control import DistanceControl

import rclpy
import argparse

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Distance Control Node')
    parser.add_argument('--truck_id', type=int, help='Truck ID')
    parsed_args, _ = parser.parse_known_args()
    distance_control = DistanceControl(truck_id=parsed_args.truck_id)
    rclpy.spin(distance_control)
    distance_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()