from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('plant_identification')
    return LaunchDescription([Node(
        package='plant_identification', executable='plant_identifier',
        name='plant_identifier', output='screen',
        parameters=[os.path.join(pkg, 'config', 'plant_id_params.yaml')]
    )])
