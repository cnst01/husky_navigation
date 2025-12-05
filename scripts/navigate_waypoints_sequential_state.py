#!/usr/bin/env python3

import rclpy
from yasmin import State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateWaypointsSequentialState(State):
    """Estado que navega por waypoints sequencialmente (um por um)
    
    Isto garante que o robô atinja cada waypoint antes de ir para o próximo,
    evitando problemas com NavigateThroughPoses que pode processar apenas
    o primeiro waypoint.
    """
    
    def __init__(self, namespace=None):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])
        import os
        self.namespace = namespace if namespace else os.environ.get('ROBOT_NAMESPACE', 'a200_0000')
        self.action_name = f"{self.namespace}/navigate_to_pose"
        self.current_waypoint_idx = 0
        
    def execute(self, blackboard: Blackboard) -> str:
        if not hasattr(blackboard, 'waypoints') or not blackboard.waypoints:
            node = rclpy.create_node('nav_seq_logger')
            node.get_logger().error("Waypoints não carregados no blackboard")
            node.destroy_node()
            return ABORT
        
        waypoints = blackboard.waypoints
        node = rclpy.create_node('navigate_sequential')
        
        try:
            # Cria client de ação para NavigateToPose
            action_client = ActionClient(node, NavigateToPose, self.action_name)
            
            if not action_client.wait_for_server(timeout_sec=10.0):
                node.get_logger().error(f"Servidor {self.action_name} não disponível")
                return ABORT
            
            node.get_logger().info(f"Iniciando navegação sequencial por {len(waypoints)} waypoints")
            
            # Navega por cada waypoint sequencialmente
            for idx, waypoint in enumerate(waypoints):
                node.get_logger().info(
                    f"Waypoint {idx + 1}/{len(waypoints)}: "
                    f"({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}, {waypoint.pose.position.z:.2f})"
                )
                
                # Cria goal para um único waypoint
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = waypoint
                
                # Envia goal e aguarda resultado
                future = action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(node, future, timeout_sec=300.0)
                
                goal_handle = future.result()
                if not goal_handle.accepted:
                    node.get_logger().error(f"Waypoint {idx + 1} rejeitado")
                    return ABORT
                
                # Aguarda o resultado do goal
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, result_future, timeout_sec=300.0)
                result = result_future.result()
                
                # NavigateToPose retorna status 0 para sucesso
                if result.status == 4:  # SUCCEEDED
                    node.get_logger().info(f"✅ Waypoint {idx + 1} atingido com sucesso")
                else:
                    node.get_logger().error(f"❌ Falha ao atingir waypoint {idx + 1} (status: {result.status})")
                    return ABORT
            
            node.get_logger().info(f"✅ Todos os {len(waypoints)} waypoints atingidos com sucesso")
            self.current_waypoint_idx = 0
            return SUCCEED
            
        except Exception as e:
            node.get_logger().error(f"Erro durante navegação sequencial: {e}")
            return ABORT
        finally:
            node.destroy_node()
