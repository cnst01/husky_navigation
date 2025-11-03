#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import sys


class Nav2GoalSender(Node):
    def __init__(self, namespace='a200_0000'):
        super().__init__('nav2_goal_sender')
        
        self.namespace = namespace
        action_name = f'{namespace}/navigate_to_pose'
        
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(f'Conectado ao servidor: {action_name}')

    def send_goal(self, x, y, yaw=0.0):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor de navegação não disponível')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(float(yaw))
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f'Goal enviado: ({x}, {y}, {yaw} rad)')
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeitado')
            return

        self.get_logger().info('Navegação iniciada')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Navegação concluída')
        rclpy.shutdown()

    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print("Uso: ros2 run husky_navigation send_goal <x> <y> [yaw]")
        print("Exemplo: ros2 run husky_navigation send_goal 2.0 1.5 0.0")
        return 1
    
    try:
        x, y = sys.argv[1], sys.argv[2]
        yaw = sys.argv[3] if len(sys.argv) > 3 else '0.0'
        
        goal_sender = Nav2GoalSender()
        
        if goal_sender.send_goal(x, y, yaw):
            rclpy.spin(goal_sender)
            
    except KeyboardInterrupt:
        print("\nOperação cancelada")
    except Exception as e:
        print(f"Erro: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())