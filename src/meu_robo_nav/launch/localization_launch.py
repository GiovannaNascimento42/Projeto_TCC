import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('meu_robo_nav')

    map_file = os.path.join(
        pkg_dir,
        'config',
        'meu_mapa_odom_teste.yaml'
    )

    params_file = os.path.join(
        pkg_dir,
        'config',
        'nav2_params.yaml'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'yaml_filename': map_file
            }
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl'
                ]
            }
        ]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])
