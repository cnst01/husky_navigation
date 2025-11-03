#!/usr/bin/env python3

import rclpy
import yaml
from yasmin import State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from geometry_msgs.msg import PoseStamped
import os
from ament_index_python.packages import get_package_share_directory

class GetWaypointsState(State):
    """Estado que carrega waypoints de arquivo YAML para o blackboard"""
    
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
    def execute(self, blackboard: Blackboard) -> str:
        # Cria um nó ROS temporário para logging
        node = rclpy.create_node('get_waypoints_state')
        
        try:
            # Caminho para o arquivo de configuração
            package_share_dir = get_package_share_directory('husky_navigation')
            config_file = os.path.join(package_share_dir, 'config', 'waypoints.yaml')
            
            node.get_logger().info(f"Carregando waypoints de: {config_file}")
            
            # Verifica se o arquivo existe
            if not os.path.exists(config_file):
                node.get_logger().error(f"Arquivo de configuração não encontrado: {config_file}")
                return ABORT
            
            # Carrega e parseia o arquivo YAML
            with open(config_file, 'r') as file:
                config_data = yaml.safe_load(file)
            
            if not config_data or 'waypoints' not in config_data:
                node.get_logger().error("Estrutura YAML inválida: falta seção 'waypoints'")
                return ABORT
            
            if 'poses' not in config_data['waypoints']:
                node.get_logger().error("Estrutura YAML inválida: falta seção 'waypoints.poses'")
                return ABORT
            
            # Converte waypoints para PoseStamped
            waypoints = []
            for i, pose_data in enumerate(config_data['waypoints']['poses']):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                
                # Usa tempo atual do ROS
                clock = rclpy.clock.Clock()
                pose_stamped.header.stamp = clock.now().to_msg()
                
                # Posição
                pose_stamped.pose.position.x = float(pose_data['position']['x'])
                pose_stamped.pose.position.y = float(pose_data['position']['y'])
                pose_stamped.pose.position.z = float(pose_data['position']['z'])
                
                # Orientação
                pose_stamped.pose.orientation.x = float(pose_data['orientation']['x'])
                pose_stamped.pose.orientation.y = float(pose_data['orientation']['y'])
                pose_stamped.pose.orientation.z = float(pose_data['orientation']['z'])
                pose_stamped.pose.orientation.w = float(pose_data['orientation']['w'])
                
                waypoints.append(pose_stamped)
                node.get_logger().info(f"Waypoint {i+1}: ({pose_stamped.pose.position.x:.2f}, "
                                     f"{pose_stamped.pose.position.y:.2f})")
            
            # Salva no blackboard
            blackboard.waypoints = waypoints
            blackboard.config = config_data.get('navigation', {})
            
            node.get_logger().info(f"✅ {len(waypoints)} waypoints carregados com sucesso")
            return SUCCEED
            
        except Exception as e:
            node.get_logger().error(f"❌ Erro ao carregar waypoints: {str(e)}")
            return ABORT
        finally:
            node.destroy_node()