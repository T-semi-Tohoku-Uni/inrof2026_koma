import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros
from pathlib import Path

import math

def generate_launch_description():
    inrof2026_koma_package_dir = get_package_share_directory("inrof2026_koma")
    simulation_package_dir = get_package_share_directory("simulation")
    komarm_package_dir = get_package_share_directory("komarm")

    # Get workspace dir
    ws_root = Path(komarm_package_dir).parents[3]
    src_path = str(ws_root / 'src')
    models_path = str(ws_root / 'src' / 'simulation' / 'models')

    rviz_config = str(Path(inrof2026_koma_package_dir) / "config" / "default.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
                ':',
                src_path,
                ':',
                models_path,
            ],
        ),
        rviz,
    ]) 