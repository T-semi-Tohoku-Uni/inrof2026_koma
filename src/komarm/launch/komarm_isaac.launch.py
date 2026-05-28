import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
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

    # Get workspace dir
    ws_root = Path(komarm_package_dir).parents[3]
    src_path = str(ws_root / 'src')
    models_path = str(ws_root / 'src' / 'simulation' / 'models')

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

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r -s 4 {world_file_path}'])]
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

    joint_manager = Node(
        package="komarm",
        executable="dummy_joint",
        output="screen",
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

    catch_influence = Node(
        package="komarm",
        executable="catch_influence",
        output="screen",
        parameters=[
            {
                "model_path": weight_path
            }
        ],
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
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
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[
                libtorch_lib,
                ':',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        ),
        robot_state_publisher_node,
        # rviz,
        joint_manager,
        hand_pose,
        static_from_map_to_odom,
        catch_influence,
    ]) 
