#!/usr/bin/env python3

import rclpy
from yasmin import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from rclpy.node import Node
import sys
import os

# Adiciona o caminho para importar os estados locais
sys.path.append(os.path.dirname(__file__))

from get_waypoints_state import GetWaypointsState
from get_waypoints_state import GetMissionState
from navigate_waypoints_sequential_state import NavigateWaypointsSequentialState

class WaypointNavigationSM(StateMachine):
    """Máquina de estados para navegação por waypoints"""
    
    def __init__(self, use_mission: bool = False, mission_file: str = None):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])

        # Escolhe qual estado irá obter os waypoints: do YAML/arquivo padrão ou
        # diretamente de um JSON de missão (GetMissionState)
        if use_mission:
            get_state = GetMissionState(filename=mission_file) if mission_file else GetMissionState()
        else:
            get_state = GetWaypointsState()

        # Adiciona estados à máquina
        self.add_state(
            "GET_WAYPOINTS",
            get_state,
            transitions={
                SUCCEED: "NAVIGATE_WAYPOINTS",
                ABORT: ABORT
            }
        )

        self.add_state(
            "NAVIGATE_WAYPOINTS",
            NavigateWaypointsSequentialState(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT,
                CANCEL: CANCEL
            }
        )

def main():
    # Inicializa ROS sem argumentos para evitar conflito com parâmetros
    # Detecta argumentos da linha de comando para selecionar modo de missão
    use_mission = False
    mission_file = None
    for arg in sys.argv[1:]:
        if arg == '--mission':
            use_mission = True
        elif arg.startswith('--mission-file='):
            use_mission = True
            mission_file = arg.split('=', 1)[1]

    rclpy.init(args=sys.argv)
    
    # Cria nó ROS 2 padrão
    node = Node("waypoint_navigation_sm")
    
    # Cria e inicia a máquina de estados, passando flags de missão ao construtor
    sm = WaypointNavigationSM(use_mission=use_mission, mission_file=mission_file)

    if use_mission:
        node.get_logger().info(f"Modo missão ativado, usando arquivo: {mission_file or 'config/mission1.json'}")
    
    # Publica FSM para visualização (opcional)
    try:
        from yasmin_viewer import YasminViewer
        YasminViewer(sm, "WAYPOINT_NAVIGATION")
        node.get_logger().info("Visualizador YASMIN ativado")
    except ImportError:
        node.get_logger().warn("yasmin_viewer não instalado, visualização não disponível")
    
    # Executa a máquina de estados
    try:
        node.get_logger().info("Iniciando máquina de estados de waypoints...")
        outcome = sm()
        node.get_logger().info(f"🏁 Máquina de estados finalizada com outcome: {outcome}")
        
    except KeyboardInterrupt:
        node.get_logger().info("Interrompido pelo usuário")
        if sm.is_running():
            sm.cancel_state()
    except Exception as e:
        node.get_logger().error(f"Erro na máquina de estados: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()