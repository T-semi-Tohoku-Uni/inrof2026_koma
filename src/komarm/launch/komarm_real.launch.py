import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros

import math

def generate_launch_description():
    inrof2026_koma_package_dir = get_package_share_directory("inrof2026_koma")
    simulation_package_dir = get_package_share_directory("simulation")
    komarm_package_dir = get_package_share_directory("komarm")

    x = 0.25
    y = 0.25
    z = 0.40

    world_file_path = os.path.join(
        simulation_package_dir,
        "worlds", 
        "field.world"
    )
    komarm_urdf_path = os.path.join(
        komarm_package_dir,
        "urdf",
        "komarm.urdf"
    )

    with open(komarm_urdf_path, "r") as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(
        "../meshes/",
        f"package://komarm/meshes/",
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
                "servo_1_min": -math.pi/2,
                "servo_1_max":  math.pi/2,

                "servo_2_min": 0.0,
                "servo_2_max": math.pi,
                "is_servo_2_reverse": True,

                "servo_3_min": 0.0,
                "servo_3_max": math.pi,

                "servo_4_min": -math.pi/4.0,
                "servo_4_max": math.pi/2.0,
                "is_servo_4_reverse": True,

                "servo_5_min": -math.pi,
                "servo_5_max": math.pi,
            }
        ]
    )

    hand_pose = Node(
        package="komarm",
        executable="hand_pose",
        output="screen",
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_manager,
        # hand_pose,
        static_from_map_to_odom,
    ]) 