#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Incluye demo.launch.py del paquete de MoveIt
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("legrobot_moveit_config"), "launch", "demo.launch.py"])
        )
    )

    # GUI monitor (separada de RViz/Gazebo GUI)
    monitor_gui = Node(
        package="lesson_urdf",
        executable="moveit_monitor_gui",
        output="screen",
    )

    # Delay para que ya exista /joint_states (MoveIt / controllers)
    delayed_gui = TimerAction(period=4.0, actions=[monitor_gui])

    return LaunchDescription([
        moveit_demo,
        delayed_gui,
    ])
