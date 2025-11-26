#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import sys


class NavigationDiagnostics(Node):
    def __init__(self):
        super().__init__('navigation_diagnostics')
        
        self.declare_parameter('namespace', 'a200_0000')
        namespace = self.get_parameter('namespace').value
        
        # Subscrever posição do robô
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{namespace}/odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Diagnósticos iniciados para namespace: {namespace}')
        self.get_logger().info('Aguardando tópicos de pose e odometria...')

    def pose_callback(self, msg):
        self.get_logger().info(
            f'Pose inicial: '
            f'x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'z={msg.pose.pose.position.z:.2f}, '
            f'frame={msg.header.frame_id}'
        )

    def odom_callback(self, msg):
        self.get_logger().info(
            f'Odometria: '
            f'x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'frame={msg.header.frame_id}',
            throttle_duration_sec=5.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    namespace = 'a200_0000'
    if len(sys.argv) > 1:
        namespace = sys.argv[1]
    
    diagnostics = NavigationDiagnostics()
    diagnostics.set_parameters([rclpy.parameter.Parameter('namespace', value=namespace)])
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
