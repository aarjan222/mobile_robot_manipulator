#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


def generate_launch_description():

    # ***** GAZEBO ***** #
    # DECLARE Gazebo WORLD file:
    irb120_ros2_gazebo = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'),
        'worlds',
        'irb120.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': irb120_ros2_gazebo}.items(),
    )

    EE_no = "false"
    EE_schunk = "true"

    # ***** ROBOT DESCRIPTION ***** #
    # ABB-IRB120 Description file package:
    irb120_description_path = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'))
    # ABB-IRB120 ROBOT urdf file path:
    xacro_file = os.path.join(irb120_description_path,
                              'urdf',
                              'irb120.urdf.xacro')

    # Generate ROBOT_DESCRIPTION for ABB-IRB120:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "EE_no": EE_no,
        "EE_schunk": EE_schunk,
    })
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'irb120'],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    # Joint trajectory controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["irb120_controller", "-c", "/controller_manager"],
    )
    # End effector controllers:
    # Schunk EGP-64:
    egp64left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_left_controller",
                   "-c", "/controller_manager"],
    )
    egp64right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_right_controller",
                   "-c", "/controller_manager"],
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        gazebo,
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    joint_trajectory_controller_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_controller_spawner,
                on_exit=[
                    egp64left_controller_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=egp64left_controller_spawner,
                on_exit=[
                    egp64right_controller_spawner,
                ]
            )
        ),

    ])
