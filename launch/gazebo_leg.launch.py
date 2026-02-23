#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _absolutize_package_meshes(robot_xml: str, pkg_name: str, pkg_share_dir: str) -> str:
    pkg_share_dir = os.path.abspath(pkg_share_dir)
    if not pkg_share_dir.endswith("/"):
        pkg_share_dir += "/"
    robot_xml = robot_xml.replace(f"package://{pkg_name}/", f"file://{pkg_share_dir}")
    robot_xml = robot_xml.replace(f"package://{pkg_name}", f"file://{pkg_share_dir[:-1]}")
    return robot_xml


def generate_launch_description():
    pkg_name = "lesson_urdf"
    pkg_share = get_package_share_directory(pkg_name)

    urdf_path = os.path.join(pkg_share, "urdf", "planar_3dof_gazebo.urdf")
    controllers_yaml = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    robot_xml = _read_file(urdf_path)
    robot_xml = robot_xml.replace("__CONTROLLERS_YAML__", controllers_yaml)
    robot_xml = _absolutize_package_meshes(robot_xml, pkg_name, pkg_share)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_xml}],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(get_package_share_directory("gazebo_ros"), "worlds", "empty.world"),
            "verbose": "true",
        }.items(),
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
            "-z", "0.35",   # <-- MÁS ALTO para evitar contacto inicial / explosión
        ],
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawner_pos = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_spawners = TimerAction(period=3.0, actions=[spawner_jsb, spawner_pos])

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        delayed_spawners,
    ])