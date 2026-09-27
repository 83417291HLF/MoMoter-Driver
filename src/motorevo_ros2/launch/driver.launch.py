from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("motorevo_ros2"))
    default_config = str(share / "config" / "motorevo_driver.yaml")
    bundled_sdk = str(share / "vendor" / "x86_64" / "libusb_fdcan.so")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("library_path", default_value=bundled_sdk),
        Node(
            package="motorevo_ros2",
            executable="motorevo_node",
            name="motorevo_driver",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {"transport.library_path": LaunchConfiguration("library_path")},
            ],
        ),
    ])

