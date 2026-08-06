import json

import numpy as np

from ft_fb_leaderarm.contract import SAMPLE_HZ, SCHEMA_VERSION
from ft_fb_leaderarm.validate_dataset import validate_dataset


def write_episode(path, zero_set_id):
    samples = 32
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "accepted": True,
        "free_space_only": True,
        "robot_side": "right",
        "sample_hz": SAMPLE_HZ,
        "zero_set_id": zero_set_id,
        "payload_id": "payload_v1",
        "controller_config_hash": "controller_v1",
        "ft_frame": "aft_sensor2",
        "observer_input_frame": "right_base_link",
        "zero_pose_deg": [5.5, 52.0, 112.0, 28.0, -107.0, -35.0],
        "duration_s": (samples - 1) / SAMPLE_HZ,
    }
    np.savez_compressed(
        path,
        metadata=np.asarray(json.dumps(metadata)),
        features=np.zeros((samples, 18), dtype=np.float32),
        raw_wrench=np.zeros((samples, 6), dtype=np.float32),
        stamp_s=np.arange(samples, dtype=np.float64) / SAMPLE_HZ + 1.0,
    )


def test_dataset_validator_requires_and_splits_independent_zero_sets(tmp_path):
    for index in range(3):
        write_episode(tmp_path / f"episode_{index}.npz", f"tare_{index}")
    report = validate_dataset(tmp_path, seed=7)
    assert report["passed"]
    assert report["episode_count"] == 3
    assert report["zero_set_group_count"] == 3
    assert all(report["splits"][name] for name in ("train", "validation", "test"))
