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
from navigate_waypoints_state import NavigateWaypointsState

class WaypointNavigationSM(StateMachine):
    """Máquina de estados para navegação por waypoints"""
    
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])
        
        # Adiciona estados à máquina
        self.add_state(
            "GET_WAYPOINTS",
            GetWaypointsState(),
            transitions={
                SUCCEED: "NAVIGATE_WAYPOINTS",
                ABORT: ABORT
            }
        )
        
        self.add_state(
            "NAVIGATE_WAYPOINTS",
            NavigateWaypointsState(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT,
                CANCEL: CANCEL
            }
        )

def main():
    # Inicializa ROS sem argumentos para evitar conflito com parâmetros
    rclpy.init(args=sys.argv)
    
    # Cria nó ROS 2 padrão
    node = Node("waypoint_navigation_sm")
    
    # Cria e inicia a máquina de estados
    sm = WaypointNavigationSM()
    
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