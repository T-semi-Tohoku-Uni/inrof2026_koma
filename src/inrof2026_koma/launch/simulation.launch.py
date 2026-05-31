import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import launch_ros

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

    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r 4 {world_file_path}'])]
    )


    # Set up koma urdf path
    with open(koma_urdf_path, "r") as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(
        "../meshes/",
        f"package://inrof2026_koma/inrof2026_koma_urdf/meshes/",
    )

    # Add gazebo joint plugin
    joint_state_plugin = """
    <gazebo>
        <plugin
        filename="ignition-gazebo-joint-state-publisher-system"
        name="ignition::gazebo::systems::JointStatePublisher">
        <topic>/joint_states</topic>
        <joint_name>Revolute_1</joint_name>
        <joint_name>Revolute_2</joint_name>
        <joint_name>Revolute_3</joint_name>
        <joint_name>Revolute_4</joint_name>
        <joint_name>Revolute_5</joint_name>
        <joint_name>Revolute_6</joint_name>
        </plugin>
    </gazebo>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        joint_state_plugin + "\n</robot>",
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
    <link name="laser_scan_link"/>

    <joint name="base_link_to_laser_scan_link" type="fixed">
        <parent link="base_link"/>
        <child link="laser_scan_link"/>
        <origin xyz="0.0765 0 -0.030" rpy="0 0 0"/>
    </joint>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        laser_scan_link + "\n</robot>",
    )

    laser_scan_plugin = """
    <gazebo reference="laser_scan_link">
        <sensor name="ldlidar" type="gpu_lidar">
            <ignition_frame_id>laser_scan_link</ignition_frame_id>
            <topic>/ldlidar_node/scan</topic>
            <update_rate>10</update_rate>

            <ray>
                <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1.0</resolution>
                    <min_angle>-1.3</min_angle>
                    <max_angle> 1.3</max_angle>
                </horizontal>
                <vertical>
                    <samples>1</samples>
                    <resolution>0.1</resolution>
                    <min_angle>0.0</min_angle>
                    <max_angle>0.0</max_angle>
                </vertical>
                </scan>
                <range>
                <min>0.02</min>
                <max>12.0</max>
                <resolution>0.015</resolution>
                </range>
            </ray>

            <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
                <render_engine>ogre</render_engine>
            </plugin>

            <alwaysOn>true</alwaysOn>
            <visualize>true</visualize>
        </sensor>
    </gazebo>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        laser_scan_plugin + "\n</robot>",
    )

    odom_plugin = """
    <gazebo>
        <plugin filename="libignition-gazebo-velocity-control-system.so"
                name="ignition::gazebo::systems::VelocityControl">
        <topic>/cmd_vel</topic>
        </plugin>
        <plugin filename="libignition-gazebo-odometry-publisher-system.so"
            name="ignition::gazebo::systems::OdometryPublisher">

        <odom_frame>odom</odom_frame>
        <robot_base_frame>base_footprint</robot_base_frame>
        <odom_publish_frequency>10</odom_publish_frequency>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <dimensions>3</dimensions>
        <gaussian_noise>0.0</gaussian_noise>
        </plugin>
    </gazebo>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        odom_plugin + "\n</robot>",
    )

    joint_position_controller_plugin = """
    <gazebo>
        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute_1</joint_name>
        <topic>/komarm/revolute_1/cmd_pos</topic>
        <p_gain>1.1</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.04</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute_2</joint_name>
        <topic>/komarm/revolute_2/cmd_pos</topic>
        <p_gain>1.1</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.04</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute_3</joint_name>
        <topic>/komarm/revolute_3/cmd_pos</topic>
        <p_gain>1.1</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.04</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute_4</joint_name>
        <topic>/komarm/revolute_4/cmd_pos</topic>
        <p_gain>1.1</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.04</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute_5</joint_name>
        <topic>/komarm/revolute_5/cmd_pos</topic>
        <p_gain>1.1</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.04</d_gain>
        </plugin>

        <plugin
        filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
        <joint_name>Revolute_6</joint_name>
        <topic>/komarm/revolute_6/cmd_pos</topic>
        <p_gain>1.1</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.04</d_gain>
        </plugin>
    </gazebo>
    """
    robot_desc = robot_desc.replace(
        "</robot>",
        joint_position_controller_plugin + "\n</robot>",
    )
    params = {"robot_description": robot_desc}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
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
            
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            
            "/komarm/revolute_1/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_2/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_3/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_4/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_5/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
            "/komarm/revolute_6/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
        ],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        remappings=[('clock', '/world/inrof/clock')]
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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy2Vel_node = Node(
        package="localization",
        executable="joy2vel",
        name="joy2vel",
        output="screen"
    )

    static_from_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        remappings=[('clock', '/world/inrof/clock')]
    )

    mcl = Node(
        package="localization",
        executable="mcl",
        name="mcl",
        parameters=[{
            "map_path": os.path.join(inrof2026_koma_package_dir, "map/")
        }],
        output="screen",
        remappings=[('clock', '/world/inrof/clock')]
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        models_path_env,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        rviz,
        map_server,
        static_from_map_to_odom,
        joy_node,
        joy2Vel_node,
        mcl
    ])