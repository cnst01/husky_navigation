#!/usr/bin/env python3

import rclpy
from yasmin_ros import ActionState
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from nav2_msgs.action import NavigateThroughPoses

class NavigateWaypointsState(ActionState):
    """Estado que executa navegação por waypoints usando Nav2"""
    
    def __init__(self, namespace='a200_0000'):
        action_name = f"{namespace}/navigate_through_poses"
        
        super().__init__(
            NavigateThroughPoses,
            action_name,
            self.create_goal,
            result_handler=self.result_handler,
            outcomes=[SUCCEED, ABORT, CANCEL]
        )
        
    def create_goal(self, blackboard: Blackboard) -> NavigateThroughPoses.Goal:
        if not hasattr(blackboard, 'waypoints') or not blackboard.waypoints:
            raise Exception("Waypoints não carregados no blackboard")
            
        goal = NavigateThroughPoses.Goal()
        goal.poses = blackboard.waypoints
        
        # Log via nó temporário
        node = rclpy.create_node('nav_goal_logger')
        node.get_logger().info(f"Iniciando navegação com {len(blackboard.waypoints)} waypoints")
        node.destroy_node()
        
        return goal
        
    def result_handler(self, blackboard, result) -> str:
        # Log via nó temporário
        node = rclpy.create_node('nav_result_logger')
        
        if result:
            node.get_logger().info("✅ Navegação por waypoints concluída")
            outcome = SUCCEED
        else:
            node.get_logger().error("❌ Navegação por waypoints falhou")
            outcome = ABORT
            
        node.destroy_node()
        return outcome