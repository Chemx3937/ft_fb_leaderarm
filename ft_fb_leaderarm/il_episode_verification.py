#!/usr/bin/env python3
"""Verify the persisted physical-FT contract of one IL episode."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

from .feedback_authorization import file_sha256


SCHEMA_VERSION = 1
ANALYSIS_TYPE = "physical_ft_il_episode_v1"
STAGES = (0.0, 0.40, 1.00)
ARRAYS = {
    "raw_wrench": ("ft/wrench_raw.zarr", (6,)),
    "raw_timestamp": ("ft/wrench_time_stamps.zarr", ()),
    "free_space_prediction": (
        "contact/free_space_wrench_prediction.zarr", (6,)),
    "contact_residual": ("contact/contact_wrench.zarr", (6,)),
    "contact_state": ("contact/contact_state.zarr", ()),
    "source_timestamp": ("contact/source_time_stamps.zarr", ()),
    "receive_timestamp": ("contact/receive_time_stamps.zarr", ()),
}


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

    shapes = {}
    for name, (relative, tail) in ARRAYS.items():
        try:
            shape = _zarr_shape(episode, relative)
            if shape[0] <= 0 or shape[1:] != tail:
                raise RuntimeError(
                    f"expected non-empty (*,{','.join(map(str, tail))}) shape, got {shape}"
                )
            shapes[name] = shape
        except RuntimeError as exc:
            failures.append(f"{name}: {exc}")

    def require_same_length(names, label):
        lengths = [shapes[name][0] for name in names if name in shapes]
        if len(lengths) == len(names) and len(set(lengths)) != 1:
            failures.append(f"{label} arrays have different row counts")

    require_same_length(("raw_wrench", "raw_timestamp"), "raw FT")
    require_same_length(
        (
            "free_space_prediction", "contact_residual", "contact_state",
            "source_timestamp", "receive_timestamp",
        ),
        "contact observation",
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "expected_feedback_gain_scale_contract": expected_stage,
        "sources": {
            "episode": str(episode),
            "episode_meta_sha256": file_sha256(meta_path),
            "model": str(model),
            "model_sha256": model_hash,
        },
        "arrays": {
            name: {"path": ARRAYS[name][0], "shape": list(shape)}
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
