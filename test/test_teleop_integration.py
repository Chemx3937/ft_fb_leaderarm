import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_ft_nodes_share_controller_state_topic():
    topic = "/contact_state/observer_input"
    for name in ("collector", "observer"):
        config = yaml.safe_load(
            (PACKAGE_ROOT / f"config/{name}.yaml").read_text()
        )
        parameters = config[next(iter(config))]["ros__parameters"]
        source = (PACKAGE_ROOT / f"ft_fb_leaderarm/{name}_node.py").read_text()
        assert parameters["observer_input_topic"] == topic
        assert f'"observer_input_topic": "{topic}"' in source


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
    assert config["tau_fb_contact_ramp_up_ms"] > 0.0
    source = (PACKAGE_ROOT / "src/single_impedance_wrench_feedback.cpp").read_text()
    assert "tau_fb_contact_scale_ * contact_wrench" in source


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


def test_free_space_gui_is_local_and_requires_collection_before_current(tmp_path):
    path = PACKAGE_ROOT / "scripts/ft_free_space_collection_gui.py"
    source = path.read_text()
    tree = ast.parse(source, filename=str(path))
    helper = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_collector_is_collecting"
    )
    namespace = {}
    exec(
        compile(
            ast.fix_missing_locations(
                ast.Module(body=[helper], type_ignores=[])),
            str(path), "exec"),
        namespace,
    )
    is_collecting = namespace["_collector_is_collecting"]
    assert is_collecting({"collecting": True})
    assert not is_collecting({"collecting": False})
    assert not is_collecting(None)
    recording_helper = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_collector_is_recording"
    )
    recording_namespace = {}
    exec(
        compile(
            ast.fix_missing_locations(
                ast.Module(body=[recording_helper], type_ignores=[])),
            str(path), "exec"),
        recording_namespace,
    )
    is_recording = recording_namespace["_collector_is_recording"]
    assert is_recording({"recording": True})
    assert not is_recording({"recording": False})
    helpers = {
        node.name: node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in ("_dataset_fingerprint", "_pipeline_arguments")
    }
    helper_namespace = {"Path": Path}
    exec(
        compile(
            ast.fix_missing_locations(
                ast.Module(body=list(helpers.values()), type_ignores=[])),
            str(path), "exec"),
        helper_namespace,
    )
    pipeline_arguments = helper_namespace["_pipeline_arguments"]
    assert pipeline_arguments("validate", "/data", "/report.json") == (
        "ft_free_space_validate",
        ["--data-dir", "/data", "--output", "/report.json"],
    )
    assert pipeline_arguments("train", "/data", "/model") == (
        "ft_free_space_train",
        ["--data-dir", "/data", "--output-dir", "/model"],
    )
    first = tmp_path / "first.npz"
    first.write_bytes(b"1")
    before = helper_namespace["_dataset_fingerprint"](tmp_path)
    first.write_bytes(b"changed")
    assert helper_namespace["_dataset_fingerprint"](tmp_path) != before
    assert 'collector = "/ft_free_space_collector/"' in source
    assert '"1": collector+"start_episode"' in source
    assert '"2": collector+"stop_episode"' in source
    assert 'if key == "1" and (not teleop_fresh or teleop_state != "IDLE"):' in source
    assert 'if key in ("c", "t", "o") and (' in source
    assert "Teleoperation 차단" in source
    assert "QProcess" in source
    assert "self.validated_dataset" in source
    assert "self.node.pending_services" in source
    assert 'key in "1ctozr"' in source
    assert "/chem_acp_raw_data_collection" not in source
    assert "/bae_r/observer_input" not in source
    imported_modules = {
        imported.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for imported in node.names
    }
    imported_modules.update(
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    )
    assert not any(
        module == "fb_leaderarm" or module.startswith("fb_leaderarm.")
        for module in imported_modules
    )

    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    launch = (PACKAGE_ROOT / "launch/collect_free_space_gui.launch.py").read_text()
    assert "scripts/ft_free_space_collection_gui.py" in cmake
    assert 'package="ft_fb_leaderarm"' in launch
    assert 'executable="ft_free_space_collection_gui.py"' in launch
    assert 'parameters=[{"data_dir": LaunchConfiguration("output_dir")}]' in launch
    assert 'DeclareLaunchArgument("start_teleop", default_value="false")' in launch
    assert 'DeclareLaunchArgument("record_only_fast", default_value="true")' in launch
    assert '"record_only_fast",' in launch
    assert 'condition=IfCondition(LaunchConfiguration("start_teleop"))' in launch
    assert 'executable="ft_fb_leader_single_impedance_teleop"' in launch
    assert '"feedback_source": "off"' in launch
    assert '"keyboard_input_enabled": False' in launch
    assert 'package="fb_leaderarm"' not in launch


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
    assert "scripts/ft_feedback_onset_evaluate" in cmake
    assert "scripts/ft_free_space_validate" in cmake
    assert "DIRECTORY config launch document" in cmake


def test_teleop_status_exposes_feedback_stage_for_il_recorder():
    source = (PACKAGE_ROOT / "src/single_impedance_keyboard_fsm.cpp").read_text()
    assert r'\"feedback_gain_scale_contract\":' in source


def test_feedback_il_gui_launch_binds_model_and_stage_to_recorder():
    source = (
        PACKAGE_ROOT / "launch/ft_feedback_leader_data_collection.launch.py"
    ).read_text()
    ast.parse(source)
    assert "ft_feedback_leader_teleop.launch.py" in source
    assert '"--model-sha256", model_hash' in source
    assert '"--feedback-gain-scale-contract", str(stage)' in source
    assert (
        'DeclareLaunchArgument("require_output_mount", default_value="true")'
        in source
    )
    assert 'package="fb_leaderarm"' in source
    assert 'executable="feedback_leaderarm_data_collection_gui.py"' in source
