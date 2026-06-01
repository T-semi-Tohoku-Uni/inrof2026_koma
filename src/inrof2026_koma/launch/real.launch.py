import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

import xacro
import math
import random

def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.30
    theta = math.pi/2

    # get each package dir
    inrof2026_koma_package_dir = get_package_share_directory("inrof2026_koma")
    simulation_package_dir = get_package_share_directory("simulation")

    # get file path
    world_file_path = os.path.join(
        simulation_package_dir,
        "worlds", 
        "field.world"
    )
    map_server_config_path = os.path.join(
        inrof2026_koma_package_dir,
        "map",
        "map.yaml"
    )
    rviz_config_path = os.path.join(
        inrof2026_koma_package_dir,
        "config",
        "default.rviz"
    )
    xacro_file_path = os.path.join(
        simulation_package_dir,
        "urdf", 
        "robot.xacro"
    )
    lifecycle_nodes = ['map_server']


    doc = xacro.process_file(xacro_file_path, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # nav2 map_server
    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {'yaml_filename': map_server_config_path},
        ],
    )

    # tf transfromer
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes}],
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    uart_bridge = Node(
        package="urat_bridge",
        executable="motor",
        output="screen"
    )

    joy2vel = Node(
        package="localization",
        executable="joy2vel",
        output="screen"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        node_robot_state_publisher,
        map_server_cmd,
        start_lifecycle_manager_cmd,
        static_from_map_to_odom,
        uart_bridge,
        joy2vel,
        joy_node
    ])