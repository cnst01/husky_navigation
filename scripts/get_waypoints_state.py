#!/usr/bin/env python3

import rclpy
import yaml
import json
import math
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


class GetMissionState(State):
    """Estado que carrega uma missão a partir de um arquivo JSON para o blackboard

    Formato esperado (exemplo em `config/mission1.json`):
    {
      "arm": true,
      "takeoff_altitude": -5.0,
      "yaw": 0.0,
      "waypoints": [ {"x":0.0, "y":0.0, "z":-5.0}, ... ],
      "wp_hold_cycles": 50,
      "auto_land": true,
      "disarm_after_land": true
    }
    """

    def __init__(self, filename: str = None):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.filename = filename

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        # yaw around Z axis
        half = yaw * 0.5
        qz = math.sin(half)
        qw = math.cos(half)
        return (0.0, 0.0, qz, qw)

    def execute(self, blackboard: Blackboard) -> str:
        node = rclpy.create_node('get_mission_state')

        try:
            # permite sobrescrever o caminho via blackboard: blackboard.mission_file
            if hasattr(blackboard, 'mission_file') and blackboard.mission_file:
                mission_file = blackboard.mission_file
            elif self.filename:
                mission_file = self.filename
            else:
                package_share_dir = get_package_share_directory('husky_navigation')
                mission_file = os.path.join(package_share_dir, 'config', 'mission1.json')

            node.get_logger().info(f"Carregando missão de: {mission_file}")

            if not os.path.exists(mission_file):
                node.get_logger().error(f"Arquivo de missão não encontrado: {mission_file}")
                return ABORT

            with open(mission_file, 'r') as f:
                mission = json.load(f)

            # Validações básicas
            if 'waypoints' not in mission or not isinstance(mission['waypoints'], list):
                node.get_logger().error("Estrutura JSON inválida: falta 'waypoints' como lista")
                return ABORT

            # Converte waypoints em PoseStamped
            waypoints = []
            clock = rclpy.clock.Clock()
            for i, wp in enumerate(mission['waypoints']):
                pose = PoseStamped()
                pose.header.frame_id = mission.get('frame_id', 'map')
                pose.header.stamp = clock.now().to_msg()

                # campos x,y,z esperados
                pose.pose.position.x = float(wp.get('x', 0.0))
                pose.pose.position.y = float(wp.get('y', 0.0))
                pose.pose.position.z = float(wp.get('z', 0.0))

                # Orientação: usa yaw global da missão se houver, caso contrário 0
                yaw = float(mission.get('yaw', 0.0))
                qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                waypoints.append(pose)
                node.get_logger().info(f"Waypoint {i+1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})")

            # Preenche o blackboard com campos úteis
            blackboard.mission = mission
            blackboard.waypoints = waypoints
            blackboard.arm = bool(mission.get('arm', False))
            blackboard.takeoff_altitude = float(mission.get('takeoff_altitude', 0.0))
            blackboard.yaw = float(mission.get('yaw', 0.0))
            blackboard.wp_hold_cycles = int(mission.get('wp_hold_cycles', 0))
            blackboard.auto_land = bool(mission.get('auto_land', False))
            blackboard.disarm_after_land = bool(mission.get('disarm_after_land', False))

            node.get_logger().info(f"✅ Missão carregada: {len(waypoints)} waypoints, arm={blackboard.arm}, takeoff_alt={blackboard.takeoff_altitude}")
            return SUCCEED

        except Exception as e:
            node.get_logger().error(f"Erro ao carregar missão: {e}")
            return ABORT
        finally:
            node.destroy_node()