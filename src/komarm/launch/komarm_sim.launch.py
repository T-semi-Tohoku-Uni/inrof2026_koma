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
        f"package://komarm/inrof2026_koma_urdf/meshes/",
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

    # Add gazebo joint plugin
    joint_state_plugin = """
    <gazebo>
        <plugin
        filename="ignition-gazebo-joint-state-publisher-system"
        name="ignition::gazebo::systems::JointStatePublisher">
        <topic>/komarm/gazebo_joint_states</topic>
        <joint_name>Revolute 12</joint_name>
        <joint_name>Revolute 11</joint_name>
        <joint_name>Revolute 7</joint_name>
        <joint_name>Revolute 8</joint_name>
        <joint_name>Revolute 9</joint_name>
        </plugin>
    </gazebo>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        joint_state_plugin + "\n</robot>",
    )

    joint_position_controller_plugin = """
    <gazebo>
        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute 12</joint_name>
        <topic>/komarm/revolute_12/cmd_pos</topic>
        <p_gain>30.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>2.0</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute 11</joint_name>
        <topic>/komarm/revolute_11/cmd_pos</topic>
        <p_gain>50.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>2.0</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute 7</joint_name>
        <topic>/komarm/revolute_7/cmd_pos</topic>
        <p_gain>10.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.5</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute 8</joint_name>
        <topic>/komarm/revolute_8/cmd_pos</topic>
        <p_gain>15.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.5</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute 9</joint_name>
        <topic>/komarm/revolute_9/cmd_pos</topic>
        <p_gain>0.001</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>1.0</d_gain>
        </plugin>
    </gazebo>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        joint_position_controller_plugin + "\n</robot>",
    )

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
            '/world/inrof/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            
            "/komarm/gazebo_joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            
            "/komarm/revolute_12/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_11/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_7/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_8/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_9/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            ],
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

    hand_pose = Node(
        package="komarm",
        executable="hand_pose",
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

    catch_influence = Node(
        package="komarm",
        executable="catch_influence",
        output="screen",
        remappings=[('clock', '/world/inrof/clock')]
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
        gazebo_node,
        robot_state_publisher_node,
        gz_spawn_entity,
        rviz,
        joint_manager,
        hand_pose,
        bridge,
        static_from_map_to_odom,
        catch_influence,
    ]) 