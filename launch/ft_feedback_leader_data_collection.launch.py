"""Launch feedback teleop with the package-owned IL recorder and GUI."""

from datetime import datetime
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ft_fb_leaderarm.feedback_authorization import file_sha256


def _required_path(context, name, kind="file", executable=False):
    value = LaunchConfiguration(name).perform(context).strip()
    expanded = Path(value).expanduser()
    path = (
        Path(os.path.abspath(os.fspath(expanded)))
        if executable
        else expanded.resolve()
    )
    valid = path.is_file() if kind == "file" else path.is_dir()
    if not value or not valid:
        raise RuntimeError(f"{name} is not a valid {kind}: {path}")
    if executable and not os.access(path, os.X_OK):
        raise RuntimeError(f"{name} is not executable: {path}")
    return path


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _pythonpath_with_system_qt():
    paths = [
        item
        for item in os.environ.get("PYTHONPATH", "").split(os.pathsep)
        if item
    ]
    paths.append("/usr/lib/python3/dist-packages")
    return os.pathsep.join(dict.fromkeys(paths))


def _setup(context):
    share = Path(get_package_share_directory("ft_fb_leaderarm"))
    teleop_launch = share / "launch/ft_feedback_leader_teleop.launch.py"
    recorder_python = _required_path(
        context, "recorder_python", executable=True)
    recorder_config = _required_path(context, "recorder_config")
    output_dir = _required_path(context, "data_output_dir", "directory")
    model_path = _required_path(context, "model_path")
    if _boolean(context, "require_output_mount"):
        mount = output_dir
        while not os.path.ismount(mount):
            mount = mount.parent
        if mount == Path(mount.anchor):
            raise RuntimeError(
                "data_output_dir must use dedicated mounted storage"
            )
    session = LaunchConfiguration("data_session_name").perform(context).strip()
    if not session or Path(session).name != session:
        raise RuntimeError("data_session_name must be one safe path component")

    feedback_enabled = _boolean(context, "learned_feedback_enable")
    try:
        requested_gain = float(
            LaunchConfiguration("feedback_gain_scale").perform(context)
        )
    except ValueError as exc:
        raise RuntimeError("feedback_gain_scale must be numeric") from exc
    stage = requested_gain if feedback_enabled else 0.0
    if stage not in (0.0, 0.40, 1.00):
        raise RuntimeError("feedback stage must be exactly 0.0, 0.40, or 1.00")
    model_hash = file_sha256(model_path)
    d435_flag = (
        "--enable-d435"
        if _boolean(context, "enable_d435")
        else "--disable-d435"
    )

    recorder_command = [
        str(recorder_python),
        "-m", "ft_fb_leaderarm.il_data_recorder",
        "--config-yaml", str(recorder_config),
        d435_flag,
        "--output-dir", str(output_dir),
        "--session-name", session,
        "--control-mode", "ros",
        "--model-sha256", model_hash,
        "--feedback-gain-scale-contract", str(stage),
    ]
    validation = subprocess.run(
        recorder_command + ["--validate-config-only"],
        text=True,
        capture_output=True,
        check=False,
    )
    if validation.returncode != 0:
        raise RuntimeError(
            "recorder preflight failed before hardware startup:\n"
            + validation.stdout
            + validation.stderr
        )
    gui_environment = dict(os.environ)
    gui_environment["PYTHONPATH"] = _pythonpath_with_system_qt()
    gui_validation = subprocess.run(
        [
            sys.executable,
            "-c",
            "import PyQt5, rclpy, contact_observer_msgs",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=gui_environment,
    )
    if gui_validation.returncode != 0:
        raise RuntimeError(
            "GUI dependency preflight failed before hardware startup:\n"
            + gui_validation.stdout
            + gui_validation.stderr
        )

    teleop_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "observer_config", "leader_config", "smooth_teleop_config", "model_path",
            "zero_set_confirmed", "zero_set_id", "payload_id",
            "controller_config_hash", "learned_feedback_enable",
            "feedback_gain_scale", "feedback_authorization",
            "leader_stale_timeout", "smooth_teleop_enable",
            "keyboard_input_enabled",
        )
    }
    gui_log_dir = output_dir / session / "collection_logs" / datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(teleop_launch)),
            launch_arguments=teleop_arguments.items(),
        ),
        ExecuteProcess(
            cmd=recorder_command,
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="ft_fb_leaderarm",
            executable="ft_feedback_leader_data_collection_gui.py",
            name="feedback_leaderarm_data_collection_gui",
            output="screen",
            additional_env={"PYTHONPATH": gui_environment["PYTHONPATH"]},
            parameters=[{
                "collection_log_dir": str(gui_log_dir),
                "observer_diagnostics_topic": "/ft_contact_observer/diagnostics",
                "observer_input_topic": "/contact_state/observer_input",
            }],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("ft_fb_leaderarm"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "recorder_python", default_value="/home/vision/venv_act/bin/python"
        ),
        DeclareLaunchArgument(
            "recorder_config",
            default_value=str(share / "config/il_data_collection.yaml"),
        ),
        DeclareLaunchArgument("data_output_dir"),
        DeclareLaunchArgument("data_session_name"),
        DeclareLaunchArgument("require_output_mount", default_value="true"),
        DeclareLaunchArgument("enable_d435", default_value="false"),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument(
            "observer_config", default_value=str(share / "config/observer.yaml")
        ),
        DeclareLaunchArgument(
            "leader_config",
            default_value=str(share / "config/single_impedance_leader_damping.yaml"),
        ),
        DeclareLaunchArgument(
            "smooth_teleop_config",
            default_value=str(
                share / "config/single_impedance_leader_smooth_teleop.yaml"
            ),
        ),
        DeclareLaunchArgument("zero_set_confirmed", default_value="false"),
        DeclareLaunchArgument("zero_set_id", default_value=""),
        DeclareLaunchArgument("payload_id"),
        DeclareLaunchArgument("controller_config_hash"),
        DeclareLaunchArgument("learned_feedback_enable", default_value="false"),
        DeclareLaunchArgument("feedback_gain_scale", default_value="0.40"),
        DeclareLaunchArgument("feedback_authorization", default_value=""),
        DeclareLaunchArgument("leader_stale_timeout", default_value="0.020"),
        DeclareLaunchArgument("smooth_teleop_enable", default_value="true"),
        DeclareLaunchArgument("keyboard_input_enabled", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
