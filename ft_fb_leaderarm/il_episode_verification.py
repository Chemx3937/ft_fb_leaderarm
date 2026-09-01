#!/usr/bin/env python3
"""Verify the persisted physical-FT contract of one IL episode."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import numpy as np
import zarr

from .feedback_authorization import file_sha256


SCHEMA_VERSION = 2
ANALYSIS_TYPE = "physical_ft_il_episode_v2"
STAGES = (0.0, 0.40, 1.00)
ARRAYS = {
    "joint": ("robot/joint_deg.zarr", (6,)),
    "joint_timestamp": ("robot/joint_time_stamps.zarr", ()),
    "hand_joint": ("robot/hand_joint.zarr", (15,)),
    "hand_timestamp": ("robot/hand_joint_time_stamps.zarr", ()),
    "command_pose": ("robot/command_pose_se3.zarr", (4, 4)),
    "command_timestamp": ("robot/command_time_stamps.zarr", ()),
    "current_pose": ("robot/controller_current_pose_se3.zarr", (4, 4)),
    "current_pose_timestamp": (
        "robot/controller_current_pose_time_stamps.zarr", ()),
    "command_quat_pose": ("robot/command_quat_pose_se3.zarr", (4, 4)),
    "command_quat_timestamp": (
        "robot/command_quat_time_stamps.zarr", ()),
    "raw_wrench": ("ft/wrench_raw.zarr", (6,)),
    "raw_timestamp": ("ft/wrench_time_stamps.zarr", ()),
    "jt_tared_wrench": ("ft/jt_tared_wrench.zarr", (6,)),
    "jt_tared_timestamp": (
        "ft/jt_tared_wrench_time_stamps.zarr", ()),
    "jt_filtered_wrench": ("ft/jt_tared_filtered_wrench.zarr", (6,)),
    "jt_filtered_timestamp": (
        "ft/jt_tared_filtered_wrench_time_stamps.zarr", ()),
    "free_space_prediction": (
        "contact/free_space_wrench_prediction.zarr", (6,)),
    "contact_residual": ("contact/contact_wrench.zarr", (6,)),
    "contact_state": ("contact/contact_state.zarr", ()),
    "contact_valid": ("contact/contact_valid.zarr", ()),
    "contact_model_ready": ("contact/contact_model_ready.zarr", ()),
    "source_timestamp": ("contact/source_time_stamps.zarr", ()),
    "receive_timestamp": ("contact/receive_time_stamps.zarr", ()),
    "source_sequence": ("contact/source_sequences.zarr", ()),
    "prediction_age": ("contact/prediction_age_ms.zarr", ()),
}

REQUIRED_CONFIG_FLAGS = (
    "enable_d435",
    "record_hand",
    "record_current_pose",
    "record_cmd_quat_pose",
    "record_ft_wrench_raw",
    "record_jt_tared_wrench",
    "record_jt_tared_filtered_wrench",
    "record_contact_observation",
)


def _load_json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"failed to read JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return value


def _zarr_shape(episode, relative):
    array = episode / relative
    metadata = array / ".zarray"
    value = _load_json(metadata)
    shape = value.get("shape")
    if (
        not isinstance(shape, list)
        or not shape
        or any(not isinstance(item, int) or item < 0 for item in shape)
    ):
        raise RuntimeError(f"invalid Zarr shape: {metadata}")
    if not any(
        path.is_file() and not path.name.startswith(".") for path in array.iterdir()
    ):
        raise RuntimeError(f"Zarr array has no data chunks: {array}")
    return tuple(shape)


def _validate_timestamp_values(episode, relative, expected_rows):
    try:
        values = np.asarray(
            zarr.open(str(episode / relative), mode="r")[:],
            dtype=np.float64,
        ).reshape(-1)
    except Exception as exc:
        raise RuntimeError(
            f"failed to read timestamp Zarr array: {episode / relative}: {exc}"
        ) from exc
    if values.size != expected_rows:
        raise RuntimeError(
            f"timestamp row count changed while reading: "
            f"{values.size} != {expected_rows}"
        )
    if not np.isfinite(values).all():
        raise RuntimeError("timestamp array contains non-finite values")
    if values.size > 1 and np.any(np.diff(values) <= 0.0):
        raise RuntimeError("timestamps are not strictly increasing")


def _camera_contract(meta):
    config = meta.get("recorder_config")
    if not isinstance(config, dict):
        raise RuntimeError("episode recorder_config is missing")
    missing = [name for name in REQUIRED_CONFIG_FLAGS if name not in config]
    if missing:
        raise RuntimeError(
            "episode recorder_config is missing: " + ", ".join(missing)
        )
    required_true = REQUIRED_CONFIG_FLAGS[1:]
    disabled = [name for name in required_true if config[name] is not True]
    if disabled:
        raise RuntimeError(
            "feedback IL recorder contract is disabled: " + ", ".join(disabled)
        )

    try:
        color_tail = (
            int(config["color_height"]), int(config["color_width"]), 3)
        depth_tail = (
            color_tail[:2]
            if config.get("align_depth_to_color") is True
            else (int(config["depth_height"]), int(config["depth_width"]))
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("episode camera dimensions are invalid") from exc
    if min(color_tail[:2] + depth_tail) <= 0:
        raise RuntimeError("episode camera dimensions must be positive")

    expected_roles = {"camera_0": {"model": "D405"}}
    if config["enable_d435"] is True:
        expected_roles["camera_1"] = {"model": "D435"}
    elif config["enable_d435"] is not False:
        raise RuntimeError("enable_d435 must be a boolean")
    roles = meta.get("camera_roles")
    if not isinstance(roles, dict) or set(roles) != set(expected_roles):
        raise RuntimeError("episode camera_roles do not match enable_d435")

    contract = {}
    directories = []
    for role, expected in expected_roles.items():
        value = roles.get(role)
        if not isinstance(value, dict) or value.get("model") != expected["model"]:
            raise RuntimeError(f"{role} must identify {expected['model']}")
        directory = f"{role}_{expected['model']}"
        directories.append(directory)
        prefix = role
        contract.update({
            f"{prefix}_rgb": (f"{directory}/rgb.zarr", color_tail),
            f"{prefix}_rgb_timestamp": (
                f"{directory}/rgb_time_stamps.zarr", ()),
            f"{prefix}_rgb_hardware_timestamp": (
                f"{directory}/rgb_hardware_time_stamps_ms.zarr", ()),
            f"{prefix}_rgb_frame_number": (
                f"{directory}/rgb_frame_numbers.zarr", ()),
            f"{prefix}_depth": (f"{directory}/depth.zarr", depth_tail),
            f"{prefix}_depth_timestamp": (
                f"{directory}/depth_time_stamps.zarr", ()),
            f"{prefix}_depth_hardware_timestamp": (
                f"{directory}/depth_hardware_time_stamps_ms.zarr", ()),
            f"{prefix}_depth_frame_number": (
                f"{directory}/depth_frame_numbers.zarr", ()),
        })
    return contract, directories, bool(config["enable_d435"])


def analyze_il_episode(episode_path, model_path, expected_stage):
    episode = Path(episode_path).expanduser().resolve()
    model = Path(model_path).expanduser().resolve()
    if not episode.is_dir():
        raise RuntimeError(f"episode directory is missing: {episode}")
    if not model.is_file():
        raise RuntimeError(f"model file is missing: {model}")
    if not any(math.isclose(expected_stage, stage) for stage in STAGES):
        raise RuntimeError("expected stage must be exactly 0.0, 0.40, or 1.00")

    meta_path = episode / "meta.json"
    meta = _load_json(meta_path)
    model_hash = file_sha256(model)
    failures = []
    if meta.get("model_sha256") != model_hash:
        failures.append("episode model_sha256 does not match the selected model")
    try:
        recorded_stage = float(meta.get("feedback_gain_scale_contract"))
    except (TypeError, ValueError):
        recorded_stage = math.nan
    if not math.isfinite(recorded_stage) or not math.isclose(
        recorded_stage, expected_stage, abs_tol=1.0e-9
    ):
        failures.append("episode feedback stage is missing or mismatched")
    if meta.get("writer_error"):
        failures.append("episode has a writer error")
    if meta.get("interruption_reason"):
        failures.append("episode was interrupted")

    camera_directories = []
    d435_enabled = None
    try:
        camera_arrays, camera_directories, d435_enabled = _camera_contract(meta)
    except RuntimeError as exc:
        camera_arrays = {}
        failures.append(str(exc))
    if d435_enabled is False and (episode / "camera_1_D435").exists():
        failures.append("D435 data exists although enable_d435 is false")

    array_contract = {**ARRAYS, **camera_arrays}
    shapes = {}
    for name, (relative, tail) in array_contract.items():
        try:
            shape = _zarr_shape(episode, relative)
            if shape[0] <= 0 or shape[1:] != tail:
                raise RuntimeError(
                    f"expected non-empty (*,{','.join(map(str, tail))}) shape, got {shape}"
                )
            shapes[name] = shape
            if "timestamp" in name:
                _validate_timestamp_values(episode, relative, shape[0])
        except RuntimeError as exc:
            failures.append(f"{name}: {exc}")

    def require_same_length(names, label):
        lengths = [shapes[name][0] for name in names if name in shapes]
        if len(lengths) == len(names) and len(set(lengths)) != 1:
            failures.append(f"{label} arrays have different row counts")

    require_same_length(("joint", "joint_timestamp"), "robot joint")
    require_same_length(("hand_joint", "hand_timestamp"), "robot hand")
    require_same_length(
        ("command_pose", "command_timestamp"), "command pose")
    require_same_length(
        ("current_pose", "current_pose_timestamp"), "current pose")
    require_same_length(
        ("command_quat_pose", "command_quat_timestamp"),
        "quaternion command pose",
    )
    require_same_length(("raw_wrench", "raw_timestamp"), "raw FT")
    require_same_length(
        ("jt_tared_wrench", "jt_tared_timestamp"), "JT tared wrench")
    require_same_length(
        ("jt_filtered_wrench", "jt_filtered_timestamp"),
        "JT filtered wrench",
    )
    require_same_length(
        (
            "free_space_prediction", "contact_residual", "contact_state",
            "contact_valid", "contact_model_ready", "source_timestamp",
            "receive_timestamp", "source_sequence", "prediction_age",
        ),
        "contact observation",
    )
    for role in ("camera_0", "camera_1"):
        if f"{role}_rgb" not in shapes:
            continue
        require_same_length(
            (
                f"{role}_rgb", f"{role}_rgb_timestamp",
                f"{role}_rgb_hardware_timestamp", f"{role}_rgb_frame_number",
            ),
            f"{role} RGB",
        )
        require_same_length(
            (
                f"{role}_depth", f"{role}_depth_timestamp",
                f"{role}_depth_hardware_timestamp",
                f"{role}_depth_frame_number",
            ),
            f"{role} depth",
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "expected_feedback_gain_scale_contract": expected_stage,
        "camera_mode": {
            "d435_enabled": d435_enabled,
            "directories": camera_directories,
        },
        "sources": {
            "episode": str(episode),
            "episode_meta_sha256": file_sha256(meta_path),
            "model": str(model),
            "model_sha256": model_hash,
        },
        "arrays": {
            name: {"path": array_contract[name][0], "shape": list(shape)}
            for name, shape in shapes.items()
        },
        "failures": failures,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Verify a saved IL episode without commanding hardware"
    )
    parser.add_argument("--episode", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--expected-stage", required=True, type=float, choices=STAGES
    )
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    try:
        args = parse_args(argv)
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite IL episode report: {output}")
        report = analyze_il_episode(args.episode, args.model, args.expected_stage)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(("PASS" if report["passed"] else "FAIL") + f": {output}")
        for failure in report["failures"]:
            print(f"- {failure}")
        return 0 if report["passed"] else 2
    except Exception as exc:
        print(f"ERROR: IL episode verification failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
