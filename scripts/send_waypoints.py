#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import sys


class WaypointNavigator(Node):
    def __init__(self, namespace='a200_0000'):
        super().__init__('waypoint_navigator')
        
        self.namespace = namespace
        action_name = f'{namespace}/navigate_through_poses'
        
        self._action_client = ActionClient(self, NavigateThroughPoses, action_name)
        self.get_logger().info(f'Waypoint Navigator inicializado: {action_name}')

    def send_waypoints(self, waypoints):
        """Envia uma lista de waypoints para navegação sequencial"""
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor de waypoints não disponível')
            return False

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f'{len(waypoints)} waypoints enviados')
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints rejeitados')
            return

        self.get_logger().info('Navegação por waypoints iniciada')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_pose
        navigation_time = feedback.navigation_time.sec
        
        self.get_logger().info(
            f'Waypoint {feedback.current_waypoint}: '
            f'Posição ({current_waypoint.pose.position.x:.1f}, '
            f'{current_waypoint.pose.position.y:.1f}), '
            f'Tempo: {navigation_time}s',
            throttle_duration_sec=3.0
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navegação por waypoints concluída')
        rclpy.shutdown()

    def create_pose(self, x, y, yaw=0.0):
        """Cria uma PoseStamped a partir de coordenadas e yaw"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(float(yaw) / 2.0),
            w=math.cos(float(yaw) / 2.0)
        )
        return pose


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print("Uso: ros2 run husky_navigation send_waypoints <x1,y1,yaw1> <x2,y2,yaw2> ...")
        print("Exemplo: ros2 run husky_navigation send_waypoints 1.0,1.0,0.0 2.0,2.0,1.57 3.0,1.0,0.0")
        print("Formato: x,y,yaw_radianos (yaw opcional, padrão=0.0)")
        return 1
    
    try:
        navigator = WaypointNavigator()
        
        # Parse dos waypoints
        waypoints = []
        for i, arg in enumerate(sys.argv[1:]):
            coords = arg.split(',')
            x = coords[0]
            y = coords[1]
            yaw = coords[2] if len(coords) > 2 else '0.0'
            
            pose = navigator.create_pose(x, y, yaw)
            waypoints.append(pose)
            
            print(f'Waypoint {i+1}: ({x}, {y}, {yaw})')

        if navigator.send_waypoints(waypoints):
            rclpy.spin(navigator)
            
    except KeyboardInterrupt:
        print("\nNavegação cancelada")
    except Exception as e:
        print(f"Erro: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())