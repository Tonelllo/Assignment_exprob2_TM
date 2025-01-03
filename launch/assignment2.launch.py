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
    assignment2_exprob_tm_share = FindPackageShare(
        package='assignment2_exprob_tm').find('assignment2_exprob_tm')
    default_model_path = os.path.join(
        assignment2_exprob_tm_share, 'urdf/robot.urdf')
    default_world_path = os.path.join(
        assignment2_exprob_tm_share, 'worlds/assignment2.world')
    config_path = os.path.join(assignment2_exprob_tm_share, 'config')
    models_path = os.path.join(assignment2_exprob_tm_share, "models")
    pddl_path = os.path.join(assignment2_exprob_tm_share, "pddl")
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    gazebo_model_path = EnvironmentVariable(
        "GAZEBO_MODEL_PATH", default_value="")
    gazebo_model_path = [gazebo_model_path, ":", models_path]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc},
                    {'use_sim_time': True}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments=[
            ('autostart', 'true'),
            ('params_file', os.path.join(config_path, "nav2.yaml"))
        ]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments=[
            ('params_file', os.path.join(config_path, 'slam_toolbox.yaml'))
        ]
    )
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments=[
            ('model_file', os.path.join(pddl_path, 'domain.pddl'))]
    )

    move_to_min = Node(
        package='assignment2_exprob_tm',
        executable='move_to_min',
        name='move_to_min',
        output='screen',
        parameters=[])

    move_cmd = Node(
        package='assignment2_exprob_tm',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[])

    explore_waypoint_cmd = Node(
        package='assignment2_exprob_tm',
        executable='explore_waypoint_action_node',
        name='explore_waypoint_action_node',
        output='screen',
        parameters=[])

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'robot',
                                   '-topic', '/robot_description', '-y', '3'],
                        output='screen')

    return LaunchDescription([
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH",
                               value=gazebo_model_path),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        slam_toolbox,
        nav2_bringup,
        spawn_entity,
        plansys2_cmd,
        move_cmd,
        move_to_min,
        explore_waypoint_cmd,
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s',
                 'libgazebo_ros_factory.so', "-s", "libgazebo_ros_init.so"],
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(config_path, "rviz.rviz")],
            output='screen'),
    ])
