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

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r 4 {world_file_path}'])]
    )

    with open(komarm_urdf_path, "r") as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(
        "../meshes/",
        f"file://{komarm_package_dir}/meshes/",
    )
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

    theta = math.pi/2
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'robot',
                   '-allow_renaming', 'false',
                   '-x', str(x),
                   '-y', str(y),
                   '-z', str(z),
                   '-Y', str(theta)
                ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ldlidar_node/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/inrof/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        remappings=[('clock', '/world/inrof/clock')]
    )

    joint_manager = Node(
        package="komarm",
        executable="dummy_joint",
        output="screen",
        remappings=[('clock', '/world/inrof/clock')]
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        remappings=[('clock', '/world/inrof/clock')]
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        gazebo_node,
        robot_state_publisher_node,
        gz_spawn_entity,
        rviz,
        joint_manager,
        bridge,
        static_from_map_to_odom,
        # static_from_odom_to_baseline
    ]) 