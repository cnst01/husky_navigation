from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        # Máquina de estados de waypoints - SEM parameters
        Node(
            package='husky_navigation',
            executable='waypoint_state_machine.py',
            name='waypoint_navigation_sm',
            output='screen'
            # Removido --params-file
        ),
    ])