from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("motorevo_ros2"))
    xacro_file = str(share / "urdf" / "pa43.urdf.xacro")
    controllers = str(share / "config" / "controllers.yaml")
    bundled_sdk = str(share / "vendor" / "x86_64" / "libusb_fdcan.so")
    backend = LaunchConfiguration("backend")
    device = LaunchConfiguration("device")
    channel = LaunchConfiguration("channel")
    motor_id = LaunchConfiguration("motor_id")
    library_path = LaunchConfiguration("library_path")
    description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " backend:=", backend,
            " device:=", device,
            " library_path:=", library_path,
            " channel:=", channel,
            " motor_id:=", motor_id,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="meow_usb"),
        DeclareLaunchArgument("device", default_value="/dev/USB2CAN0"),
        DeclareLaunchArgument("library_path", default_value=bundled_sdk),
        DeclareLaunchArgument("channel", default_value="1"),
        DeclareLaunchArgument("motor_id", default_value="1"),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": description}, controllers],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        ),
    ])

