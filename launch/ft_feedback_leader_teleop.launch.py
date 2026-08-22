"""Physical-FT observer plus staged right-arm leader feedback."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ft_fb_leaderarm.feedback_authorization import (
    MAX_STALE_TIMEOUT_S,
    scaled_leader_overrides,
    validate_feedback_authorization,
)


def _as_bool(name, value):
    lowered = str(value).strip().lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _setup(context, ft_share, observer_launch):
    model_path = LaunchConfiguration("model_path").perform(context)
    leader_config = LaunchConfiguration("leader_config").perform(context)
    feedback_enabled = _as_bool(
        "learned_feedback_enable",
        LaunchConfiguration("learned_feedback_enable").perform(context),
    )
    smooth_teleop_enabled = _as_bool(
        "smooth_teleop_enable",
        LaunchConfiguration("smooth_teleop_enable").perform(context),
    )
    try:
        requested_scale = float(
            LaunchConfiguration("feedback_gain_scale").perform(context)
        )
        stale_timeout = float(
            LaunchConfiguration("leader_stale_timeout").perform(context)
        )
    except ValueError as exc:
        raise RuntimeError(
            "feedback_gain_scale and leader_stale_timeout must be numeric"
        ) from exc
    if (
        not math.isfinite(stale_timeout)
        or stale_timeout <= 0.0
        or stale_timeout > MAX_STALE_TIMEOUT_S
    ):
        raise RuntimeError(
            f"leader_stale_timeout must be finite and within "
            f"(0, {MAX_STALE_TIMEOUT_S:.3f}]"
        )
    authorization = LaunchConfiguration("feedback_authorization").perform(
        context
    ).strip()
    if feedback_enabled:
        if not authorization:
            raise RuntimeError(
                "learned_feedback_enable=true requires feedback_authorization"
            )
        validated = validate_feedback_authorization(
            authorization, model_path, requested_scale
        )
        gain_scale = float(validated["gain_scale"])
    else:
        if authorization:
            raise RuntimeError(
                "feedback_authorization must be empty while feedback is disabled"
            )
        gain_scale = 0.0

    leader_overrides = {
        "side": "right",
        # Keep the canonical observer subscription and state display alive even
        # when feedback is OFF; zero gain is the only OFF mechanism.
        "feedback_source": "contact_observer",
        "feedback_gain_scale_contract": gain_scale,
        "contact_observation_topic": "/contact_observer/right/observation",
        "contact_observation_stale_timeout": stale_timeout,
        "use_jt_wrench_feedback": False,
        "use_pre_contact_phase": False,
        "tau_fb_contact_gate_enable": False,
        "intent_generator_enabled": smooth_teleop_enabled,
        "keyboard_input_enabled": LaunchConfiguration("keyboard_input_enabled"),
    }
    leader_overrides.update(
        scaled_leader_overrides(leader_config, "right", gain_scale)
    )
    print(
        "[FT FEEDBACK] observer subscription=ON, reflected torque="
        + (f"ON at {gain_scale:.0%}" if feedback_enabled else "OFF (zero gain)")
        + ", smooth teleop="
        + ("ON" if smooth_teleop_enabled else "OFF")
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(observer_launch)),
            launch_arguments={
                "config": LaunchConfiguration("observer_config"),
                "model_path": LaunchConfiguration("model_path"),
                "zero_set_confirmed": LaunchConfiguration("zero_set_confirmed"),
                "zero_set_id": LaunchConfiguration("zero_set_id"),
                "payload_id": LaunchConfiguration("payload_id"),
                "controller_config_hash": LaunchConfiguration(
                    "controller_config_hash"
                ),
            }.items(),
        ),
        Node(
            package="ft_fb_leaderarm",
            executable="ft_fb_leader_single_impedance_teleop",
            name="leader_teleop_node",
            output="screen",
            emulate_tty=True,
            parameters=[leader_config, leader_overrides],
        ),
    ]


def generate_launch_description():
    ft_share = Path(get_package_share_directory("ft_fb_leaderarm"))
    observer_launch = ft_share / "launch/ft_contact_observer.launch.py"
    declarations = [
        DeclareLaunchArgument(
            "observer_config", default_value=str(ft_share / "config/observer.yaml")
        ),
        DeclareLaunchArgument(
            "leader_config",
            default_value=str(ft_share / "config/single_impedance_leader_damping.yaml"),
        ),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("zero_set_confirmed", default_value="false"),
        DeclareLaunchArgument("zero_set_id", default_value=""),
        DeclareLaunchArgument("payload_id"),
        DeclareLaunchArgument("controller_config_hash"),
        DeclareLaunchArgument("learned_feedback_enable", default_value="false"),
        DeclareLaunchArgument(
            "feedback_gain_scale",
            default_value="0.40",
            description="Authorized stage: exactly 0.40 or 1.00",
        ),
        DeclareLaunchArgument("feedback_authorization", default_value=""),
        DeclareLaunchArgument("leader_stale_timeout", default_value="0.020"),
        DeclareLaunchArgument("smooth_teleop_enable", default_value="true"),
        DeclareLaunchArgument("keyboard_input_enabled", default_value="true"),
    ]
    return LaunchDescription(
        declarations
        + [
            OpaqueFunction(
                function=_setup, args=[str(ft_share), str(observer_launch)]
            )
        ]
    )
