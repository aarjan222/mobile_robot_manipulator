from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    xacro_file = get_package_share_directory(
        'mobile_manipulator_body') + '/urdf' + '/robot.urdf.xacro'

    # Robot State Publisher
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': Command(['xacro', ' ', xacro_file])
                                              }])

    # Gazebo
    world_file_name = 'office_small.world'
    # world_file_name = 'slamworld.world'
    # world_file_name = 'turtlebot3_world.world'
    world = os.path.join(get_package_share_directory(
        'mobile_manipulator_body'), 'worlds', world_file_name)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen')

    # Spawn the robot in Gazebo
    spawn_entity_robot = Node(package='gazebo_ros',
                              executable='spawn_entity.py',
                              arguments=['-entity', 'robot_base',
                                         '-topic', 'robot_description'],
                              output='screen')

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity_robot,
    ])
