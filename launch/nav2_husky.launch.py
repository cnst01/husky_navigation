from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Argumentos de launch
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
    
    # Incluir localization.launch.py
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
            'use_sim_time': LaunchConfiguration('use_sim_time')
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
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        setup_path_arg,
        view_navigation_launch,
        localization_launch,
        nav2_launch
    ])