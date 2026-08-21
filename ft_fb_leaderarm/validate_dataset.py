#!/usr/bin/env python3
"""Validate collected physical-FT episodes before expensive ablation training."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import numpy as np

from .train_ablation import load_sessions, session_manifest, split_by_zero_set


def validate_dataset(data_dir, seed=7):
    sessions = load_sessions(data_dir)
    sample_hz = float(sessions[0].metadata["sample_hz"])
    splits = split_by_zero_set(sessions, seed)
    force = np.concatenate([session.wrench[:, :3] for session in sessions], axis=0)
    groups = sorted({session.group for session in sessions})
    return {
        "schema_version": 1,
        "validation_type": "physical_ft_free_space_dataset_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": True,
        "data_dir": str(Path(data_dir).expanduser().resolve()),
        "episode_count": len(sessions),
        "zero_set_group_count": len(groups),
        "zero_set_ids": groups,
        "sample_count": sum(len(session.features) for session in sessions),
        "sample_hz": sample_hz,
        "duration_s": sum(float(session.metadata["duration_s"]) for session in sessions),
        "raw_force_norm_max_n": float(np.max(np.linalg.norm(force, axis=1))),
        "contracts": {
            key: sessions[0].metadata[key]
            for key in (
                "ft_frame",
                "observer_input_frame",
                "payload_id",
                "controller_config_hash",
                "zero_pose_deg",
            )
        },
        "splits": session_manifest(splits),
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite dataset report: {output}")
        report = validate_dataset(args.data_dir, args.seed)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"GO: dataset validation passed: {output}")
        print(
            f"episodes={report['episode_count']} groups={report['zero_set_group_count']} "
            f"samples={report['sample_count']} duration_s={report['duration_s']:.3f}"
        )
        return 0
    except Exception as exc:
        print(f"NO-GO: dataset validation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
