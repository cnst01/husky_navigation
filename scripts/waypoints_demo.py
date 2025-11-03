#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math


class WaypointDemo(Node):
    def __init__(self, namespace='a200_0000'):
        super().__init__('waypoint_demo')
        self._action_client = ActionClient(self, NavigateThroughPoses, f'{namespace}/navigate_through_poses')

    def run_square_demo(self, start_x=1.0, start_y=1.0, size=2.0):
        """Demo: navegação em quadrado"""
        waypoints = [
            self.create_pose(start_x, start_y, 0.0),
            self.create_pose(start_x + size, start_y, 0.0),
            self.create_pose(start_x + size, start_y + size, 1.57),
            self.create_pose(start_x, start_y + size, 3.14),
            self.create_pose(start_x, start_y, -1.57)
        ]
        
        return self.send_waypoints(waypoints)

    def run_circle_demo(self, center_x=2.0, center_y=2.0, radius=1.5):
        """Demo: navegação em pontos circulares"""
        waypoints = []
        for i in range(8):
            angle = i * (2 * math.pi / 8)
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            yaw = angle + math.pi  # Olhar para fora do círculo
            waypoints.append(self.create_pose(x, y, yaw))
        
        return self.send_waypoints(waypoints)

    def send_waypoints(self, waypoints):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor de waypoints não disponível')
            return False

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f'Demo iniciada com {len(waypoints)} waypoints')
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Demo rejeitada')
            return

        self.get_logger().info('Demo de waypoints em execução')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Demo de waypoints concluída')
        rclpy.shutdown()

    def create_pose(self, x, y, yaw):
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


def main():
    rclpy.init()
    
    demo = WaypointDemo()
    
    print("Selecione a demo:")
    print("1. Navegação em Quadrado")
    print("2. Navegação Circular")
    print("3. Waypoints Customizados")
    
    try:
        choice = input("Opção (1-3): ").strip()
        
        if choice == '1':
            print("Iniciando demo Quadrado...")
            demo.run_square_demo()
            rclpy.spin(demo)
        elif choice == '2':
            print("Iniciando demo Circular...")
            demo.run_circle_demo()
            rclpy.spin(demo)
        elif choice == '3':
            print("Use: ros2 run husky_navigation send_waypoints.py")
        else:
            print("Opção inválida")
            
    except KeyboardInterrupt:
        print("\nDemo cancelada")
    except Exception as e:
        print(f"Erro: {e}")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()