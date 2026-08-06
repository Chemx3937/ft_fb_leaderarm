from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("ft_fb_leaderarm"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=str(share / "config/observer.yaml"),
            ),
            DeclareLaunchArgument("model_path"),
            DeclareLaunchArgument("zero_set_confirmed", default_value="false"),
            DeclareLaunchArgument("zero_set_id", default_value=""),
            DeclareLaunchArgument("payload_id"),
            DeclareLaunchArgument("controller_config_hash"),
            Node(
                package="ft_fb_leaderarm",
                executable="ft_contact_observer",
                name="ft_contact_observer",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "zero_set_confirmed": LaunchConfiguration(
                            "zero_set_confirmed"
                        ),
                        "zero_set_id": LaunchConfiguration("zero_set_id"),
                        "payload_id": LaunchConfiguration("payload_id"),
                        "controller_config_hash": LaunchConfiguration(
                            "controller_config_hash"
                        ),
                    },
                ],
            ),
        ]
    )
