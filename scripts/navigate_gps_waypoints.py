#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import sys
import yaml
import os
from pathlib import Path


class GPSWaypointNavigator(Node):
    def __init__(self, namespace='a200_0000', reference_lat=-30.0, reference_lon=-51.0):
        super().__init__('gps_waypoint_navigator')
        
        self.namespace = namespace
        action_name = f'{namespace}/navigate_through_poses'
        
        # Parâmetros GPS
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        
        # Fatores de conversão mais realistas para diferenças pequenas
        # 1 grau de latitude = ~111.32 km = 111320 m
        # 1 grau de longitude = ~111.32 * cos(latitude) km
        # Para latitudes próximas a -30°, cos(-30°) ≈ 0.866
        self.lat_to_meters = 111320.0  # metros por grau
        self.lon_to_meters = 111320.0 * 0.866  # metros por grau (ajustado para -30°)
        
        self._action_client = ActionClient(self, NavigateThroughPoses, action_name)
        self.get_logger().info(f'GPS Waypoint Navigator inicializado: {action_name}')
        self.get_logger().info(f'Referência GPS: Lat={reference_lat}, Lon={reference_lon}')
        self.get_logger().info(f'Fatores conversão: lat={self.lat_to_meters:.1f} m/°, lon={self.lon_to_meters:.1f} m/°')

    def gps_to_pose(self, latitude, longitude, yaw=0.0):
        """Converte coordenadas GPS para PoseStamped no frame 'map'"""
        x = (longitude - self.reference_lon) * self.lon_to_meters
        y = (latitude - self.reference_lat) * self.lat_to_meters
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        
        # Calcular quaternion corretamente
        yaw_rad = float(yaw)
        sin_half_yaw = math.sin(yaw_rad / 2.0)
        cos_half_yaw = math.cos(yaw_rad / 2.0)
        
        pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=sin_half_yaw,
            w=cos_half_yaw
        )
        
        self.get_logger().debug(
            f'Pose criada: pos=({x:.2f}, {y:.2f}), '
            f'quat=({pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, '
            f'{pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f})'
        )
        
        return pose

    def load_gps_waypoints(self, yaml_file):
        """Carrega waypoints GPS de um arquivo YAML"""
        try:
            # Resolver o caminho do arquivo
            yaml_path = Path(yaml_file)
            
            # Se o caminho for relativo, tentar encontrá-lo em locais comuns
            if not yaml_path.is_absolute():
                # Tentar relativo ao diretório atual
                if not yaml_path.exists():
                    # Tentar em ../config (se executado de scripts/)
                    alt_path = Path(__file__).parent.parent / 'config' / yaml_file
                    if alt_path.exists():
                        yaml_path = alt_path
                    else:
                        # Tentar como path relativo ao workspace
                        alt_path = Path.cwd() / 'src/husky_navigation/config' / yaml_file
                        if alt_path.exists():
                            yaml_path = alt_path
                        else:
                            # Tentar com o nome do arquivo apenas
                            alt_path = Path.cwd() / yaml_file
                            if alt_path.exists():
                                yaml_path = alt_path
            
            yaml_path = yaml_path.resolve()
            self.get_logger().info(f'Carregando waypoints de: {yaml_path}')
            
            if not yaml_path.exists():
                self.get_logger().error(f'Arquivo não encontrado: {yaml_path}')
                self.get_logger().error(f'Diretório atual: {Path.cwd()}')
                return None
            
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' not in data:
                self.get_logger().error("Chave 'waypoints' não encontrada no YAML")
                return None
            
            waypoints_data = data['waypoints']
            
            # Suporta dois formatos: lista ou dicionário com poses
            if isinstance(waypoints_data, list):
                gps_points = waypoints_data
            elif isinstance(waypoints_data, dict) and 'gps_points' in waypoints_data:
                gps_points = waypoints_data['gps_points']
            else:
                self.get_logger().error("Formato de waypoints não reconhecido")
                return None
            
            poses = []
            for i, point in enumerate(gps_points):
                if 'latitude' not in point or 'longitude' not in point:
                    self.get_logger().warn(f'Waypoint {i} sem latitude/longitude, pulando')
                    continue
                
                lat = float(point['latitude'])
                lon = float(point['longitude'])
                yaw = float(point.get('yaw', 0.0))
                
                pose = self.gps_to_pose(lat, lon, yaw)
                poses.append(pose)
                
                self.get_logger().info(
                    f'Waypoint {i}: GPS({lat:.6f}, {lon:.6f}) -> Map({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
                )
            
            return poses
        
        except yaml.YAMLError as e:
            self.get_logger().error(f'Erro ao parsear YAML: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Erro ao carregar waypoints: {e}')
            return None

    def send_waypoints(self, waypoints):
        """Envia waypoints para navegação sequencial"""
        
        if not waypoints:
            self.get_logger().error('Nenhum waypoint válido')
            return False
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor de waypoints não disponível')
            return False

        # Validar poses
        for i, pose in enumerate(waypoints):
            self.get_logger().info(
                f'Enviando waypoint {i}: '
                f'pos=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), '
                f'frame={pose.header.frame_id}'
            )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints
        
        self.get_logger().info(f'Meta enviada com {len(waypoints)} waypoints')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f'{len(waypoints)} waypoints GPS convertidos e enviados')
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints rejeitados')
            return

        self.get_logger().info('Navegação por waypoints GPS iniciada')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        # Atributos disponíveis no feedback de NavigateThroughPoses
        if hasattr(feedback, 'number_of_poses_remaining'):
            poses_remaining = feedback.number_of_poses_remaining
            self.get_logger().info(
                f'Poses restantes: {poses_remaining}',
                throttle_duration_sec=3.0
            )
        
        if hasattr(feedback, 'current_pose'):
            current = feedback.current_pose
            self.get_logger().info(
                f'Posição atual: ({current.pose.position.x:.2f}, {current.pose.position.y:.2f})',
                throttle_duration_sec=3.0
            )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navegação por waypoints GPS concluída')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Uso: ros2 run husky_navigation navigate_gps_waypoints <arquivo_yaml>")
        print("Opções adicionais:")
        print("  --namespace <namespace>          (padrão: a200_0000)")
        print("  --ref-lat <latitude>              (padrão: -30.0)")
        print("  --ref-lon <longitude>             (padrão: -51.0)")
        print("\nExemplo:")
        print("  ros2 run husky_navigation navigate_gps_waypoints config/gps_waypoints.yaml")
        print("  ros2 run husky_navigation navigate_gps_waypoints config/gps_waypoints.yaml --ref-lat -30.0 --ref-lon -51.0")
        return 1
    
    try:
        # Parse argumentos
        yaml_file = sys.argv[1]
        namespace = 'a200_0000'
        ref_lat = -30.0
        ref_lon = -51.0
        
        i = 2
        while i < len(sys.argv):
            if sys.argv[i] == '--namespace':
                namespace = sys.argv[i + 1]
                i += 2
            elif sys.argv[i] == '--ref-lat':
                ref_lat = float(sys.argv[i + 1])
                i += 2
            elif sys.argv[i] == '--ref-lon':
                ref_lon = float(sys.argv[i + 1])
                i += 2
            else:
                i += 1
        
        navigator = GPSWaypointNavigator(namespace=namespace, reference_lat=ref_lat, reference_lon=ref_lon)
        
        # Carregar e enviar waypoints
        waypoints = navigator.load_gps_waypoints(yaml_file)
        if waypoints:
            navigator.send_waypoints(waypoints)
            rclpy.spin(navigator)
        else:
            return 1
    
    except Exception as e:
        print(f'Erro: {e}', file=sys.stderr)
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
