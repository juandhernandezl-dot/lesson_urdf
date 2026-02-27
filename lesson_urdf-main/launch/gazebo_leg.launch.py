#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "lesson_urdf"
    pkg_share = get_package_share_directory(pkg_name)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(get_package_share_directory("gazebo_ros"), "worlds", "empty.world"),
            "verbose": "true",
        }.items(),
    )

    xacro_exec = FindExecutable(name="xacro")
    xacro_path = PathJoinSubstitution([FindPackageShare(pkg_name), "urdf", "planar_3dof.urdf.xacro"])
    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    # Pass pkg_share so xacro can build file:// paths for Gazebo
    robot_description = Command([
        xacro_exec, " ",
        xacro_path, " ",
        "pkg_share:=", pkg_share, " ",
        "controllers_yaml:=", controllers_yaml, " ",
        "mesh_scale:='1 1 1'", " ",
        "fix_to_world:=true", " ",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "planar_3dof_leglike",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "1.0",
        ],
    )

    controller_mgr = "/controller_manager"

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", controller_mgr,
            "--controller-manager-timeout", "120",
        ],
    )

    spawner_pos = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "position_controller",
            "--controller-manager", controller_mgr,
            "--controller-manager-timeout", "120",
        ],
    )

    joint_gui = Node(
        package="lesson_urdf",
        executable="joint_gui",
        output="screen",
        )

    delayed_spawners = TimerAction(period=8.0, actions=[spawner_jsb, spawner_pos])

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        delayed_spawners,
        joint_gui,
    ])