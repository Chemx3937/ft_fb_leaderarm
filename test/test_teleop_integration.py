from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_ft_teleop_is_built_from_local_sources():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    expected_sources = (
        "single_impedance_teleop_main.cpp",
        "single_impedance_teleop_node.cpp",
        "single_impedance_gravity_compensation.cpp",
        "single_impedance_pose_publisher.cpp",
        "single_impedance_wrench_feedback.cpp",
        "single_impedance_keyboard_fsm.cpp",
        "dynamixel_bus.cpp",
    )
    assert "add_executable(ft_fb_leader_single_impedance_teleop" in cmake
    for source in expected_sources:
        assert (PACKAGE_ROOT / "src" / source).is_file()
        assert source in cmake


def test_ft_teleop_config_cannot_default_to_jt_feedback():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config/single_impedance_leader_damping.yaml").read_text()
    )["leader_teleop_node"]["ros__parameters"]
    assert config["side"] == "right"
    assert config["feedback_source"] == "contact_observer"
    assert config["contact_observation_topic"] == (
        "/contact_observer/right/observation"
    )
    assert config["use_jt_wrench_feedback"] is False
    assert config["tau_fb_contact_gate_enable"] is False


def test_integrated_launch_uses_only_local_teleop_executable():
    launch = (
        PACKAGE_ROOT / "launch/ft_feedback_leader_teleop.launch.py"
    ).read_text()
    assert 'package="ft_fb_leaderarm"' in launch
    assert 'executable="ft_fb_leader_single_impedance_teleop"' in launch
    assert 'get_package_share_directory("fb_leaderarm")' not in launch
    assert 'package="fb_leaderarm"' not in launch
    assert '"feedback_source": "contact_observer"' in launch
    assert '"feedback_source": "off"' not in launch
    assert "scaled_leader_overrides" in launch
    assert 'default_value="0.40"' in launch
    assert "0.20, 0.40, 0.80" not in launch


def test_ft_feedback_csv_contains_automatic_analysis_contract():
    source = (PACKAGE_ROOT / "src/single_impedance_teleop_node.cpp").read_text()
    for column in (
        "observer_valid",
        "observer_model_ready",
        "observer_source_age_ms",
        "observer_contact_state",
        "observer_contact_score_N",
        "observer_prediction_sequence",
    ):
        assert column in source
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "scripts/ft_feedback_analyze" in cmake
    assert "scripts/ft_free_space_validate" in cmake
    assert "DIRECTORY config launch docs" in cmake
