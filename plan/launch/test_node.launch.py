from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="plan",
            executable="plan_test_node",
            name="plan_test_node",
            output="screen",
        ),
    ])
