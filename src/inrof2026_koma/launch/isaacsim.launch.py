import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import launch_ros
import re

from pathlib import Path

import xacro
import math
import random

def generate_launch_description():
    x = 0.25
    y = 0.25
    z = 0.256
    theta = math.pi/2

    # get each package dir
    inrof2026_koma_package_dir = get_package_share_directory("inrof2026_koma")
    simulation_package_dir = get_package_share_directory("simulation")
    komarm_package_dir = get_package_share_directory("komarm")
    field_package_dir = get_package_share_directory("field")

    # libtorch path
    libtorch_lib = str(Path(komarm_package_dir) / 'libtorch' / 'lib')
    weight_path = str(Path(komarm_package_dir) / "weight" / "policy.pt")

    # Get workspace dir
    ws_root = Path(komarm_package_dir).parents[3]
    src_path = str(ws_root / 'src')
    models_path = str(ws_root / 'src' / 'field' / 'models')

    print(src_path)

    # Set env
    models_path_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
            ':',
            src_path,
            ':',
            models_path,
        ],
    )

    libtorch_env = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[
            libtorch_lib,
            ':',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )

    # get file path
    world_file_path = os.path.join(
        field_package_dir,
        "worlds", 
        "field.world"
    )
    rviz_config_path = os.path.join(
        inrof2026_koma_package_dir,
        "config",
        "default.rviz"
    )
    koma_urdf_path = os.path.join(
        inrof2026_koma_package_dir,
        "inrof2026_koma_urdf",
        "urdf",
        "komarm.urdf"
    )

    ball_urdf_path = os.path.join(
        simulation_package_dir,
        "urdf",
        "ball.urdf"
    )

    # Set up koma urdf path
    with open(koma_urdf_path, "r") as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(
        "../meshes/",
        f"package://inrof2026_koma/inrof2026_koma_urdf/meshes/",
    )

    base_footprint_joint = """
    <link name="base_footprint"/>

    <joint name="base_footprint_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 0.0685" rpy="0 0 0"/>
    </joint>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        base_footprint_joint + "\n</robot>",
    )

    laser_scan_link = """
    <link name="ldlidar_base"/>
    <link name="ldlidar_link"/>

    <joint name="base_link_to_ldlidar_base" type="fixed">
        <parent link="base_link"/>
        <child link="ldlidar_base"/>
        <origin xyz="0.0765 0 -0.030" rpy="0 0 0"/>
    </joint>

    <joint name="ldlidar_base_to_scan" type="fixed">
        <parent link="ldlidar_base"/>
        <child link="ldlidar_link"/>
        <origin xyz="0 0 0" rpy="0 0 1.5708"/>
    </joint>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        laser_scan_link + "\n</robot>",
    )

    # add end effector link
    end_effector_link = """
    <link name="end_effector"/>

    <joint name="end_effector_link" type="fixed">
        <parent link="hand_unit_v3_1"/>
        <child link="end_effector"/>
        <origin xyz="0.1 0 0.02" rpy="0 0 0"/>
    </joint>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        end_effector_link + "\n</robot>",
    )
    params = {"robot_description": robot_desc}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    map_server = Node(
        package="field",
        executable="map_server",
        output="screen",
        parameters=[
            {
                "mesh_resource": "package://field/models/inrof_field/meshes/InrofField.dae"
            }
        ]
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    ) 

    catch_influence = Node(
        package="komarm",
        executable="catch_influence",
        output="screen",
        parameters=[
            {
                "model_path": weight_path,
                "default_position": [
                    0.0, -1.3, 0.0, 1.57, 0.0, 0.0
                ]
            }
        ],
    )

    odom_tf_broadcaster = Node(
        package="localization",
        executable="broadcaster",
        output="screen",
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        models_path_env,
        libtorch_env,
        node_robot_state_publisher,
        map_server,
        static_from_map_to_odom,
        catch_influence,
        odom_tf_broadcaster
        # foxglove_bridge
    ])