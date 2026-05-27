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

    # libtorch path
    libtorch_lib = str(Path(komarm_package_dir) / 'libtorch' / 'lib')
    weight_path = str(Path(komarm_package_dir) / "weight" / "policy.pt")


    x = 0.25
    y = 0.25
    z = 0.40

    world_file_path = os.path.join(
        simulation_package_dir,
        "worlds", 
        "field.world"
    )
    komarm_urdf_path = os.path.join(
        inrof2026_koma_package_dir,
        "inrof2026_koma_urdf",
        "urdf",
        "komarm.urdf"
    )

    with open(komarm_urdf_path, "r") as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(
        "../meshes/",
        f"package://inrof2026_koma/inrof2026_koma_urdf/meshes/",
    )

    # TODO: delete
    fixed_base_joint = f"""
    <link name="world"/>

    <joint name="world_to_base_link" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
        <origin xyz="{x} {y} {z}" rpy="0 0 0"/>
    </joint>
    """
    robot_desc = robot_desc.replace("</robot>", fixed_base_joint + "\n</robot>")

    params = {"robot_description": robot_desc}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
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

    hand_pose = Node(
        package="komarm",
        executable="hand_pose",
        output="screen",
        parameters=[
            {
                "target_x_min": 0.20,
                "target_x_max": 0.20,
                "target_y_min": -0.20,
                "target_y_max": 0.20,
                "target_z_min": 0.10,
                "target_z_max": 0.10,
            }
        ]
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    )

    catch_influence = Node(
        package="komarm",
        executable="catch_influence",
        output="screen",
        parameters=[
            {
                "model_path": weight_path
            }
        ]
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "address": "0.0.0.0",
                "port": 8765,
            }
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[
                libtorch_lib,
                ':',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        ),
        robot_state_publisher_node,
        joint_manager,
        hand_pose,
        static_from_map_to_odom,
        catch_influence,
        foxglove_bridge
    ]) 