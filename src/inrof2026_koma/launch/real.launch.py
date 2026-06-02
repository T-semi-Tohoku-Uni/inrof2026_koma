import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    field_package_dir = get_package_share_directory("field")
    lidar_package_dir = get_package_share_directory("ldlidar_node")

    # libtorch path
    libtorch_lib = str(Path(komarm_package_dir) / 'libtorch' / 'lib')
    weight_path = str(Path(komarm_package_dir) / "weight" / "policy.pt")

    # lidar setting
    ldlidar_params = str(Path(inrof2026_koma_package_dir) / "config" / "ldlidar_settings.yaml")
    ldlidar_launch = str(Path(lidar_package_dir) / "launch" / "ldlidar_with_mgr.launch.py")

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
    lifecycle_nodes = ['map_server']


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
    <link name="ldlidar_scan"/>

    <joint name="base_link_to_ldlidar_base" type="fixed">
        <parent link="base_link"/>
        <child link="ldlidar_base"/>
        <origin xyz="0.0765 0 -0.030" rpy="0 0 0"/>
    </joint>

    <joint name="ldlidar_base_to_scan" type="fixed">
        <parent link="ldlidar_base"/>
        <child link="ldlidar_scan"/>
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

    mcl = Node(
        package="localization",
        executable="mcl",
        name="mcl",
        parameters=[{
            "map_path": os.path.join(inrof2026_koma_package_dir, "map/")
        }],
        output="screen",
    )

    odom_tf_broadcaster = Node(
        package="localization",
        executable="broadcaster",
        output="screen",
    )

    planner = Node(
        package="planning",
        executable="path_plan",
        name="path_plan",
        output="screen",
        parameters=[{
            "map_path": os.path.join(inrof2026_koma_package_dir, "map/"),
            "initial_x": x,
            "initial_y": y,
            "initial_z": z,
            "initial_theta": theta
        }],
    )

    pursuit = Node(
        package="planning",
        executable="pursuit",
        name="pursuit",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.10,
            "max_angular_speed": 0.7,
            "max_linear_tolerance": 0.20,
            "max_theta_tolerance": 0.10,
            "max_reaching_distance": 0.05,
            "max_reaching_theta": 0.10,
            "lookahead_distance": 0.20,
            "resampleThreshold": 0.10,
            "Kp_tan": 1.0,
            "Ki_tan": 0.0,
            "Kd_tan": 0.0,
            "Kp_normal": 1.0,
            "Ki_normal": 0.00,
            "Kd_normal": 0.00,
            "Kp_theta": 1.0,
            "Ki_theta": 0.00,
            "Kd_theta": 0.00,
        }],
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
            "config_path": os.path.join(inrof2026_koma_package_dir, "config", "koma_bt.xml")
        }],
    )

    path_ball_position = Node(
        package="planning",
        executable="ball_plan",
        output="screen",
    )

    ldlidar_with_mgr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ldlidar_launch),
        launch_arguments={"params_file": ldlidar_params}.items(),
    )


    return LaunchDescription([
        models_path_env,
        libtorch_env,
        node_robot_state_publisher,
        map_server,
        static_from_map_to_odom,
        catch_influence,
        joint_manager,
        uart_bridge,
        mcl,
        planner,
        pursuit,
        # joy2vel,
        # joy_node
        ball_detect,
        bt,
        path_ball_position,
        odom_tf_broadcaster,
        ldlidar_with_mgr
    ])
