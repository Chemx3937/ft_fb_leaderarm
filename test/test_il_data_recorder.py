import json
import sys
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from ft_fb_leaderarm import il_data_recorder as recorder


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config/il_data_collection.yaml"


def _parse(monkeypatch, *extra):
    monkeypatch.setattr(
        sys,
        "argv",
        ["ft_il_data_collect", "--config-yaml", str(CONFIG), *extra],
    )
    return recorder.parse_args()


def _bare_recorder_node():
    ros_names = (
        "Node", "ContactObservation", "ObserverInput", "JointState",
        "PoseStamped", "WrenchStamped", "Float64MultiArray", "Int32",
        "String", "Trigger", "QoSProfile", "ReliabilityPolicy",
    )
    node_type = recorder.make_ros_node_class(
        {name: object for name in ros_names}
    )
    node = node_type.__new__(node_type)
    node.args = SimpleNamespace(
        record_contact_observation=True,
        observer_input_frame_id="right_base_link",
        observed_source_frames={},
    )
    node.lock = threading.RLock()
    node.robot_error = None
    node.latest_contact_observation = None
    node.latest_contact_observation_source_t = 0.0
    node.latest_contact_observation_rx_t = 0.0
    node.latest_contact_observation_sequence = None
    node.latest_contact_observation_status = None
    node.contact_observation_source_restarts = 0
    node.contact_observation_invalid_transitions = 0
    node.contact_observation_pre = deque()
    node._should_record_contact_observation_sample = lambda _stamp: True
    return node


def test_local_config_uses_d405_by_default_and_keeps_d435_optional(monkeypatch):
    args = _parse(monkeypatch)
    assert not args.enable_d435
    assert args.observer_input_topic == "/contact_state/observer_input"
    assert [item[:2] for item in recorder.configured_camera_specs(args)] == [
        (0, "D405")
    ]

    enabled = _parse(monkeypatch, "--enable-d435")
    assert enabled.enable_d435
    assert [item[:2] for item in recorder.configured_camera_specs(enabled)] == [
        (0, "D405"), (1, "D435")
    ]


def test_contact_prediction_age_accepts_fail_closed_observer_sentinels():
    assert recorder.normalize_contact_prediction_age_ms(
        -1.0, valid=False, model_ready=False
    ) == 0.0
    assert recorder.normalize_contact_prediction_age_ms(
        float("inf"), valid=False, model_ready=True
    ) == 0.0
    with pytest.raises(ValueError, match="sentinel"):
        recorder.normalize_contact_prediction_age_ms(
            -1.0, valid=True, model_ready=True
        )


def test_ros_executor_failure_is_reported_and_stops_runtime():
    stop_event = threading.Event()
    node = SimpleNamespace(executor_error="")
    executor = SimpleNamespace(
        spin=mock.Mock(side_effect=RuntimeError("callback failed"))
    )

    recorder.spin_ros_executor(executor, node, stop_event)

    assert stop_event.is_set()
    assert node.executor_error == "RuntimeError: callback failed"

    node.executor_error = ""
    recorder.spin_ros_executor(executor, node, stop_event)
    assert node.executor_error == ""


def test_recording_controller_forwards_free_space_prediction():
    controller = recorder.RecordingController.__new__(recorder.RecordingController)
    controller._enqueue_live = mock.Mock()
    wrench = recorder.np.arange(6, dtype=recorder.np.float64)
    prediction = wrench + 10.0

    controller.write_contact_observation(
        1.0, 1.1, 7, wrench, 0, True, True, 0.2, prediction
    )

    enqueue = controller._enqueue_live.call_args.args[2]
    pending_enqueue = controller._enqueue_live.call_args.args[3]()
    writer = SimpleNamespace(write_contact_observation=mock.Mock())
    enqueue(writer)
    recorder.np.testing.assert_array_equal(
        writer.write_contact_observation.call_args.args[-1], prediction
    )

    pending_writer = SimpleNamespace(write_contact_observation=mock.Mock())
    prediction[:] = -1.0
    pending_enqueue(pending_writer)
    recorder.np.testing.assert_array_equal(
        pending_writer.write_contact_observation.call_args.args[-1],
        recorder.np.arange(6, dtype=recorder.np.float64) + 10.0,
    )


def test_camera_restart_waits_for_reconnect_period(monkeypatch):
    camera = SimpleNamespace(
        camera_id=0,
        model="D405",
        restart=mock.Mock(),
        lock=threading.RLock(),
        latest_error=None,
    )
    args = SimpleNamespace(
        camera_reconnect_period_sec=5.0,
        camera_hardware_reset_after_restarts=3,
        camera_hardware_reset_settle_sec=6.0,
    )
    rows = [{"name": "camera_0_rgb", "ok": False}]
    last_restart = {}
    restart_counts = {}

    monkeypatch.setattr(recorder, "now_s", lambda: 100.0)
    recorder.maybe_restart_failed_cameras(
        rows, [camera], last_restart, restart_counts, args
    )
    camera.restart.assert_not_called()

    monkeypatch.setattr(recorder, "now_s", lambda: 106.0)
    recorder.maybe_restart_failed_cameras(
        rows, [camera], last_restart, restart_counts, args
    )
    camera.restart.assert_called_once_with(
        hardware_reset=False,
        hardware_reset_settle_sec=6.0,
    )


def test_recorder_rejects_unsafe_session_name(monkeypatch):
    with pytest.raises(SystemExit):
        _parse(monkeypatch, "--session-name", "../escaped")


def test_session_rejects_camera_contract_change_and_unfinished_episode(
    monkeypatch, tmp_path
):
    args = _parse(monkeypatch)
    session = tmp_path / "session"
    episode = session / "episode_000"
    episode.mkdir(parents=True)
    recorder.write_json(
        episode / "meta.json",
        {"recorder_config": recorder.recorder_config_snapshot(args)},
    )
    recorder.validate_session_compatibility(session, args)

    enabled = _parse(monkeypatch, "--enable-d435")
    with pytest.raises(RuntimeError, match="enable_d435"):
        recorder.validate_session_compatibility(session, enabled)

    unfinished = tmp_path / "unfinished"
    (unfinished / ".episode_003_recording").mkdir(parents=True)
    with pytest.raises(RuntimeError, match="unfinished crash-recovery"):
        recorder.validate_session_compatibility(unfinished, args)


def test_episode_writer_creates_only_enabled_camera_directories(
    monkeypatch, tmp_path
):
    for enable_d435, expected in (
        (False, ["camera_0_D405"]),
        (True, ["camera_0_D405", "camera_1_D435"]),
    ):
        extra = ("--enable-d435",) if enable_d435 else ()
        args = _parse(monkeypatch, *extra)
        args.camera_calibration = {}
        args.ft_processing_metadata = {}
        args.observed_source_frames = {}
        temp = tmp_path / f".episode_{int(enable_d435)}_recording"
        writer = recorder.EpisodeWriter(
            temp, tmp_path / f"episode_{int(enable_d435)}", args
        )
        writer.close()
        actual = sorted(
            path.name for path in temp.iterdir() if path.name.startswith("camera_")
        )
        assert actual == expected
        meta = json.loads((temp / "meta.json").read_text())
        assert sorted(meta["camera_roles"]) == [
            f"camera_{index}" for index in range(len(expected))
        ]
        assert meta["episode_frame_reference_release_ok"]


def test_validate_config_only_checks_dependencies_without_hardware(
    monkeypatch, tmp_path
):
    args = _parse(monkeypatch)
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='test'/>")
    args.follower_urdf = str(urdf)
    args.output_dir = str(tmp_path)
    args.session_name = "new_session"
    args.validate_config_only = True

    with (
        mock.patch.object(recorder, "parse_args", return_value=args),
        mock.patch.object(recorder, "load_cv2") as load_cv2,
        mock.patch.object(recorder, "load_zarr") as load_zarr,
        mock.patch.object(recorder, "load_realsense") as load_realsense,
        mock.patch.object(recorder, "load_ros2") as load_ros2,
        mock.patch.object(recorder.RealSenseCamera, "start") as camera_start,
    ):
        assert recorder.main() == 0

    load_cv2.assert_called_once_with()
    load_zarr.assert_called_once_with()
    load_realsense.assert_called_once_with()
    load_ros2.assert_called_once_with()
    camera_start.assert_not_called()


def test_recover_service_discards_pending_and_clears_error():
    node = _bare_recorder_node()
    node.startup_ready = True
    node.robot_sampler = object()
    node.cameras = [object()]
    node.last_control_error = "previous health auto-stop"
    node.controller = SimpleNamespace(
        is_recording=lambda: False,
        is_draining=lambda: False,
        finalize_pending=mock.Mock(return_value="episode_101"),
    )
    response = SimpleNamespace(success=False, message="")
    with (
        mock.patch.object(
            recorder,
            "memory_status",
            return_value={"level": "OK", "critical_reasons": []},
        ),
        mock.patch.object(recorder, "startup_status_rows", return_value=[]),
    ):
        result = node._recover_service(None, response)

    assert result is response
    assert response.success
    assert response.message == "discarded episode_101; recorder sources READY"
    assert node.last_control_error == ""
    node.controller.finalize_pending.assert_called_once_with(False)
