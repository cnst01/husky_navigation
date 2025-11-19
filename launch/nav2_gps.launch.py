from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # --- Argumentos de Launch ---
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0000',
        description='Namespace for the robot'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=os.path.expanduser('~/clearpath/'),
        description='Path to setup files'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('clearpath_nav2_demos'),
            'maps',
            'warehouse.yaml'
        ]),
        description='Full path to map file to load'
    )

    # --- Nó de conversão GPS para Pose ---
    gps_to_pose_node = Node(
        package='husky_navigation',
        executable='gps_to_pose.py',
        name='gps_to_pose',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # --- Inclusões de Launch Files ---
    
    # Incluir view_navigation.launch.py
    view_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_viz'),
                'launch',
                'view_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace')
        }.items()
    )
    
    # Incluir localization.launch.py (AMCL)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_nav2_demos'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'setup_path': LaunchConfiguration('setup_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map')
        }.items()
    )
    
    # Incluir nav2.launch.py
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_nav2_demos'),
                'launch',
                'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'setup_path': LaunchConfiguration('setup_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map')
        }.items()
    )
    
    # --- Lista de Ações de Lançamento ---
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        setup_path_arg,
        map_arg,
        
        gps_to_pose_node,  # <-- Adicionamos o nó de conversão GPS
        view_navigation_launch,
        localization_launch,
        nav2_launch
    ])