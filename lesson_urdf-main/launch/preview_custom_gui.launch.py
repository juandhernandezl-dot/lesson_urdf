import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("lesson_urdf")
    urdf_path = os.path.join(pkg, "urdf", "Pierna_entera_2.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    gui = Node(
        package="lesson_urdf",
        executable="joint_gui",
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz_config = os.path.join(pkg, "rviz", "view.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([rsp, gui, rviz])