"""
Spawn Robot Description
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='assignment2_exprob_tm').find('assignment2_exprob_tm')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot.xacro')
    default_world_path = os.path.join(test_robot_description_share, 'worlds/assignment2.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/test.rviz')
    models_path = os.path.join(test_robot_description_share, "models")

    gazebo_model_path = EnvironmentVariable(
        "GAZEBO_MODEL_PATH", default_value="")
    gazebo_model_path = [gazebo_model_path, ":", models_path]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    {'use_sim_time': False}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters = [{'use_sim_time': False}]
    )
   
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-y', '3'],
                        output='screen')

    return LaunchDescription([
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH",
                               value=gazebo_model_path),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so', "-s", "libgazebo_ros_init.so"],
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'),
    ])
