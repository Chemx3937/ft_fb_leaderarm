import hashlib
import json

import numpy as np
import zarr

from ft_fb_leaderarm.il_episode_verification import ARRAYS, analyze_il_episode


def _write_array(episode, relative, shape):
    target = episode / relative
    if "time_stamp" in relative:
        array = zarr.open(str(target), mode="w", shape=shape, dtype="f8")
        array[:] = np.arange(shape[0], dtype=np.float64)
        return
    target.mkdir(parents=True)
    (target / ".zarray").write_text(json.dumps({"shape": list(shape)}))
    (target / "0").write_bytes(b"chunk")


def _episode(tmp_path, enable_d435=False):
    episode = tmp_path / "episode_000"
    episode.mkdir()
    model = tmp_path / "model.ts"
    model.write_bytes(b"approved model")
    model_hash = hashlib.sha256(model.read_bytes()).hexdigest()
    recorder_config = {
        "enable_d435": enable_d435,
        "record_hand": True,
        "record_current_pose": True,
        "record_cmd_quat_pose": True,
        "record_ft_wrench_raw": True,
        "record_jt_tared_wrench": True,
        "record_jt_tared_filtered_wrench": True,
        "record_contact_observation": True,
        "color_width": 640,
        "color_height": 480,
        "depth_width": 640,
        "depth_height": 480,
        "align_depth_to_color": True,
    }
    camera_roles = {"camera_0": {"model": "D405", "serial": "d405"}}
    if enable_d435:
        camera_roles["camera_1"] = {"model": "D435", "serial": "d435"}
    (episode / "meta.json").write_text(json.dumps({
        "model_sha256": model_hash,
        "feedback_gain_scale_contract": 0.40,
        "writer_error": None,
        "interruption_reason": None,
        "recorder_config": recorder_config,
        "camera_roles": camera_roles,
    }))
    for relative, tail in ARRAYS.values():
        _write_array(episode, relative, (100,) + tail)
    for camera in ("camera_0_D405",) + (("camera_1_D435",) if enable_d435 else ()):
        for name, tail in (
            ("rgb.zarr", (480, 640, 3)),
            ("rgb_time_stamps.zarr", ()),
            ("rgb_hardware_time_stamps_ms.zarr", ()),
            ("rgb_frame_numbers.zarr", ()),
            ("depth.zarr", (480, 640)),
            ("depth_time_stamps.zarr", ()),
            ("depth_hardware_time_stamps_ms.zarr", ()),
            ("depth_frame_numbers.zarr", ()),
        ):
            _write_array(episode, f"{camera}/{name}", (100,) + tail)
    return episode, model


def test_il_episode_requires_complete_ft_feedback_contract(tmp_path):
    episode, model = _episode(tmp_path)
    report = analyze_il_episode(episode, model, 0.40)
    assert report["passed"]
    assert report["arrays"]["free_space_prediction"]["shape"] == [100, 6]
    assert report["camera_mode"] == {
        "d435_enabled": False,
        "directories": ["camera_0_D405"],
    }

    prediction = episode / ARRAYS["free_space_prediction"][0] / ".zarray"
    prediction.unlink()
    (episode / "meta.json").write_text("{}")
    rejected = analyze_il_episode(episode, model, 0.40)
    assert not rejected["passed"]
    assert any("free_space_prediction" in value for value in rejected["failures"])
    assert "episode model_sha256 does not match the selected model" in rejected["failures"]
    assert "episode feedback stage is missing or mismatched" in rejected["failures"]


def test_il_episode_requires_d435_only_when_enabled(tmp_path):
    episode, model = _episode(tmp_path, enable_d435=True)
    report = analyze_il_episode(episode, model, 0.40)
    assert report["passed"]
    assert report["camera_mode"]["directories"] == [
        "camera_0_D405", "camera_1_D435"
    ]

    metadata = episode / "camera_1_D435/rgb.zarr/.zarray"
    metadata.unlink()
    rejected = analyze_il_episode(episode, model, 0.40)
    assert not rejected["passed"]
    assert any("camera_1_rgb" in value for value in rejected["failures"])


def test_il_episode_rejects_unexpected_d435_data(tmp_path):
    episode, model = _episode(tmp_path)
    (episode / "camera_1_D435").mkdir()
    report = analyze_il_episode(episode, model, 0.40)
    assert not report["passed"]
    assert "D435 data exists although enable_d435 is false" in report["failures"]


def test_il_episode_rejects_nonmonotonic_timestamps(tmp_path):
    episode, model = _episode(tmp_path)
    timestamps = zarr.open(
        str(episode / ARRAYS["raw_timestamp"][0]), mode="r+"
    )
    timestamps[20] = timestamps[19]
    report = analyze_il_episode(episode, model, 0.40)
    assert not report["passed"]
    assert any(
        "raw_timestamp" in value and "strictly increasing" in value
        for value in report["failures"]
    )
