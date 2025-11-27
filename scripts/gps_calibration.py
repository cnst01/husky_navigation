#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys


class GPSCalibration(Node):
    def __init__(self):
        super().__init__('gps_calibration')
        
        import os
        self.declare_parameter('namespace', os.environ.get('ROBOT_NAMESPACE', 'a200_0000'))
        namespace = self.get_parameter('namespace').value
        
        # Subscrever posição do robô
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('=== CALIBRAÇÃO GPS ===')
        self.get_logger().info('Aguardando posição do robô no mapa...')
        self.get_logger().info('')
        self.get_logger().info('INSTRUÇÕES:')
        self.get_logger().info('1. Coloque o robô em uma posição conhecida no mapa')
        self.get_logger().info('2. Anote a posição no tópico abaixo (x, y)')
        self.get_logger().info('3. Verifique o GPS do robô (latitude, longitude)')
        self.get_logger().info('4. Use esses valores como referência:')
        self.get_logger().info('   ros2 run husky_navigation navigate_gps_waypoints gps_waypoints.yaml')
        self.get_logger().info('   --ref-lat <GPS_LAT> --ref-lon <GPS_LON>')
        self.get_logger().info('')

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(
            f'Posição no mapa: x={x:.3f}, y={y:.3f}',
            throttle_duration_sec=10.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibration = GPSCalibration()
        rclpy.spin(calibration)
    except KeyboardInterrupt:
        print('\nCalibração cancelada')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
