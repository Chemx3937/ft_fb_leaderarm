"""Launch the physical-FT collector GUI, optionally with feedback-OFF teleop."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pythonpath_with_system_qt():
    paths = [os.environ.get("PYTHONPATH", ""), "/usr/lib/python3/dist-packages"]
    return os.pathsep.join(path for path in paths if path)


def generate_launch_description():
    share = Path(get_package_share_directory("ft_fb_leaderarm"))
    collector_launch = share / "launch/collect_free_space.launch.py"
    arguments = [
        DeclareLaunchArgument(
            "config", default_value=str(share / "config/collector.yaml")),
        DeclareLaunchArgument(
            "output_dir", default_value="/home/vision/.ros/ft_fb_leaderarm/data"),
        DeclareLaunchArgument("zero_set_confirmed", default_value="false"),
        DeclareLaunchArgument("zero_set_id", default_value=""),
        DeclareLaunchArgument("payload_id"),
        DeclareLaunchArgument("controller_config_hash"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("record_only_fast", default_value="true"),
        DeclareLaunchArgument("start_teleop", default_value="false"),
        DeclareLaunchArgument(
            "leader_config",
            default_value=str(
                share / "config/single_impedance_leader_damping.yaml"
            ),
        ),
    ]
    collector_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "config", "output_dir", "zero_set_confirmed", "zero_set_id",
            "payload_id", "controller_config_hash", "auto_start",
            "record_only_fast",
        )
    }
    return LaunchDescription(
        arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(collector_launch)),
                launch_arguments=collector_arguments.items(),
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_teleop")),
                package="ft_fb_leaderarm",
                executable="ft_fb_leader_single_impedance_teleop",
                name="leader_teleop_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("leader_config"),
                    {
                        "side": "right",
                        "feedback_source": "off",
                        "keyboard_input_enabled": False,
                    },
                ],
            ),
            Node(
                package="ft_fb_leaderarm",
                executable="ft_free_space_collection_gui.py",
                name="ft_free_space_collection_gui",
                output="screen",
                parameters=[{"data_dir": LaunchConfiguration("output_dir")}],
                additional_env={"PYTHONPATH": _pythonpath_with_system_qt()},
            ),
        ]
    )
