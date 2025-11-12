import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- Declaração de Argumentos ---
    
    # Argumento para use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Argumento para o namespace do robô
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/a200_0000',
        description='Robot namespace (e.g., /a200_0000)'
    )

    # --- Configurações ---

    # Obter os valores dos argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Obter os diretórios dos pacotes necessários
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_clearpath_viz = get_package_share_directory('clearpath_viz')

    # --- Inclusão dos Launch Files ---

    # 1. Lançar o nav2.launch.py (Nav2)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'namespace': namespace  # Descomente se o nav2.launch.py também precisar do namespace
        }.items()
    )

    # 2. Lançar o slam.launch.py (SLAM)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_nav2_demos, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'namespace': namespace # Descomente se o slam.launch.py também precisar do namespace
        }.items()
    )

    # 3. Lançar o view_navigation.launch.py (RViz)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_viz, 'launch', 'view_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }.items()
    )

    # --- Geração da Descrição do Lançamento ---

    ld = LaunchDescription()

    # Adicionar os argumentos ao launch
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)

    # Adicionar os launch files incluídos
    ld.add_action(nav2_launch)
    ld.add_action(slam_launch)
    ld.add_action(rviz_launch)

    return ld