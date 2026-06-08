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
    lidar_package_dir = get_package_share_directory("ldlidar_node")

    # libtorch path
    libtorch_lib = str(Path(komarm_package_dir) / 'libtorch' / 'lib')
    weight_path = str(Path(komarm_package_dir) / "weight" / "policy.pt")

    # lidar setting
    ldlidar_params = str(Path(inrof2026_koma_package_dir) / "config" / "ldlidar_settings.yaml")
    ldlidar_launch = str(Path(lidar_package_dir) / "launch" / "ldlidar_with_mgr.launch.py")


    x = 0.25
    y = 0.25
    z = 0.25
    theta = math.pi/2.0

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


    robot_state_publisher_node = Node(
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

    joint_manager = Node(
        package="komarm",
        executable="feetech_joint",
        output="screen",
        parameters=[
            {
                "port_1_2": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.1:1.0",
                "port_3_4": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.2:1.0",
                "port_5_6": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.3:1.0",
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

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    )

    static_from_odom_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[str(x), str(y), str(z), str(theta), '0', '0', 'odom', 'base_footprint'],
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

    ball_detect = Node(
        package="ball_detection",
        executable="dbscan",
        output="screen",
        parameters=[{
            "map_path": os.path.join(inrof2026_koma_package_dir, "map/"),
        }],
    )

    bt = Node(
        package="behavior",
        executable="bt",
        output="screen",
        parameters=[{
            "config_path": os.path.join(komarm_package_dir, "config", "lift.xml")
        }],
    )

    ldlidar_with_mgr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ldlidar_launch),
        launch_arguments={"params_file": ldlidar_params}.items(),
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
        static_from_map_to_odom,
        static_from_odom_to_base_link,
        map_server,
        catch_influence,
        ball_detect,
        bt,
        ldlidar_with_mgr,
    ]) 