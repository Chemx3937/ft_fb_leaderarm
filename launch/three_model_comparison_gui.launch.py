"""Diagnostic GUI only: no robot, hand, FT driver, zero-set, or feedback launch."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def _pythonpath():
    # Keep PyTorch's typing_extensions ahead of the older system Qt packages.
    paths = ["/home/vision/venv_act/lib/python3.10/site-packages",
             os.environ.get("PYTHONPATH", ""), "/usr/lib/python3/dist-packages"]
    return os.pathsep.join(p for p in paths if p)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("session", description="Comparison session.json path"),
        DeclareLaunchArgument("python", default_value="/home/vision/venv_act/bin/python"),
        ExecuteProcess(
            cmd=[LaunchConfiguration("python"), "-m", "ft_fb_leaderarm.comparison_gui",
                 "--session", LaunchConfiguration("session")],
            output="screen",
            additional_env={"PYTHONPATH": _pythonpath()},
        ),
    ])
