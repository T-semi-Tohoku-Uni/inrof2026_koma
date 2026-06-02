import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
from pathlib import Path

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
    komarm_package_dir = get_package_share_directory("komarm")

    weight_path = str(Path(komarm_package_dir) / "weight" / "policy.pt")

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
        package="uart_bridge",
        executable="motor",
        output="screen",
        parameters=[{
            "Kp_linear": 0.1,
            "Kp_angular": 0.05,
            "max_linear_acceleration": 5.0,
            "max_angular_acceleration": 10.0
        }]
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

    joint_manager = Node(
        package="komarm",
        executable="feetech_joint",
        output="screen",
        parameters=[
            {
            #     "servo_1_min": -math.pi/2,
            #     "servo_1_max":  math.pi/2,

            #     "servo_2_min": 0.0,
            #     "servo_2_max": math.pi,
            "is_servo_2_reverse": True,
            "is_servo_3_reverse": True,
            "is_servo_4_reverse": True,
            # "is_servo_4_reverse": True,

            #     "servo_3_min": 0.0,
            #     "servo_3_max": math.pi,

            #     "servo_4_min": -math.pi/4.0,
            #     "servo_4_max": math.pi/2.0,

            #     "servo_5_min": -math.pi,
            #     "servo_5_max": math.pi,
            }
        ]
    )

    catch_influence = Node(
        package="komarm",
        executable="catch_influence",
        output="screen",
        parameters=[
            {
                "model_path": weight_path
            }
        ],
        remappings=[('clock', '/world/inrof/clock')]
    )


    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        node_robot_state_publisher,
        map_server_cmd,
        start_lifecycle_manager_cmd,
        static_from_map_to_odom,
        catch_influence,
        joint_manager,
        # uart_bridge,
        # joy2vel,
        # joy_node
    ])
