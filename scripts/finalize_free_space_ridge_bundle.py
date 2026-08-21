#!/usr/bin/env python3
"""Build and benchmark the selected train13/validation3 physical ridge bundle."""

import argparse
import hashlib
import json
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory

from ft_fb_leaderarm.contract import (
    BASE_FEATURE_DIM,
    SAMPLE_HZ,
    SCHEMA_VERSION,
    error_metrics,
)
from ft_fb_leaderarm.model import (
    BundlePredictor,
    PHYSICAL_RIDGE_ABLATION,
    PHYSICAL_RIDGE_CONTRACT,
    file_sha256,
    save_bundle,
)

import run_free_space_improvement_sweep as sweep
import run_physical_residual_validation as physical
import run_targeted6_validation_ablation as targeted


DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_ridge_short_multiscale_bundle_v3_20260822"
)
URDF_PACKAGE = "aidin_dsr_dualarm_description"
URDF_RELATIVE_PATH = "urdf/aidin_dsr_dualarm_aligned_hand.urdf"
PARITY_LIMIT_N = 5.0e-4
REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
SOURCE_FILES = (
    "package.xml",
    "ft_fb_leaderarm/contract.py",
    "ft_fb_leaderarm/model.py",
    "scripts/finalize_free_space_ridge_bundle.py",
    "scripts/run_free_space_improvement_sweep.py",
    "scripts/run_physical_residual_validation.py",
    "scripts/run_targeted6_validation_ablation.py",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--benchmark-calls", type=int, default=2000)
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def source_hashes():
    return {
        relative: file_sha256(REPOSITORY_ROOT / relative)
        for relative in SOURCE_FILES
    }


def benchmark_predictor(predictor, sessions, calls):
    if calls < 2000:
        raise ValueError("benchmark requires at least 2000 calls")
    per_session = int(np.ceil(calls / len(sessions)))
    windows = []
    offsets = np.arange(predictor.history - 1, -1, -1)
    for session in sessions:
        ends = np.linspace(
            predictor.history - 1,
            len(session.features) - 1,
            per_session,
            dtype=int,
        )
        windows.extend(session.features[ends[:, None] - offsets[None, :]])
    windows = windows[:calls]
    torch.set_num_threads(1)
    for window in windows[:100]:
        predictor.predict(window)
    durations_ms = []
    for window in windows:
        started = time.perf_counter_ns()
        predictor.predict(window)
        durations_ms.append((time.perf_counter_ns() - started) * 1.0e-6)
    values = np.asarray(durations_ms, dtype=np.float64)
    period_ms = 1000.0 / SAMPLE_HZ
    mean_ms = float(values.mean())
    p99_ms = float(np.percentile(values, 99.0))
    max_ms = float(values.max())
    return {
        "calls": calls,
        "includes": (
            "54D projection + TorchScript ridge residual + "
            "Pinocchio payload gravity"
        ),
        "mean_ms": mean_ms,
        "p99_ms": p99_ms,
        "max_ms": max_ms,
        "mean_equivalent_hz": 1000.0 / mean_ms,
        "p99_equivalent_hz": 1000.0 / p99_ms,
        "worst_equivalent_hz": 1000.0 / max_ms,
        "target_hz": SAMPLE_HZ,
        "period_ms": period_ms,
        "p99_limit_ms": 0.8 * period_ms,
        "max_limit_ms": period_ms,
        "passed": p99_ms <= 0.8 * period_ms and max_ms <= period_ms,
    }


def runtime_predictions(predictor, sessions):
    predictions = []
    for session in sessions:
        for end in range(predictor.history - 1, len(session.features)):
            predictions.append(
                predictor.predict(
                    session.features[end + 1 - predictor.history : end + 1]
                )
            )
    return np.asarray(predictions)


def self_check():
    rng = np.random.default_rng(8)
    train = {
        "x": rng.normal(size=(200, 54)).astype(np.float32),
        "y": rng.normal(size=(200, 6)).astype(np.float32),
        "group": np.zeros(200, dtype=np.int16),
        "group_names": ["train"],
    }
    validation = {
        "x": rng.normal(size=(20, 54)).astype(np.float32),
        "y": rng.normal(size=(20, 6)).astype(np.float32),
        "group": np.zeros(20, dtype=np.int16),
        "group_names": ["validation"],
    }
    result = sweep.ridge_candidate(train, validation, 1.0)
    prediction = (
        sweep.ridge_torch_model(result)(torch.from_numpy(validation["x"]))
        .detach()
        .numpy()
    )
    assert np.allclose(prediction, result["prediction"], atol=2.0e-5)


def main():
    args = parse_args()
    self_check()
    if args.self_check:
        print("free-space ridge bundle self-check: PASS")
        return
    if args.benchmark_calls < 2000:
        raise SystemExit("--benchmark-calls must be at least 2000")
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite: {output}")

    splits = targeted.load_fixed_splits()
    if set(splits) != {"train", "validation"}:
        raise RuntimeError("bundle finalizer refuses held-out/test splits")
    deltas = physical.gravity_deltas(splits)
    payload = physical.identify_payload(splits["train"], deltas)
    residual = physical.residualize(splits, deltas, payload)
    train = sweep.build_data(
        residual["train"], "short_multiscale", sweep.SEED, validation=False
    )
    validation = sweep.build_data(
        residual["validation"],
        "short_multiscale",
        sweep.SEED + 1000,
        validation=True,
        common_history=32,
    )
    result = sweep.ridge_candidate(train, validation, 1.0)
    model = sweep.ridge_torch_model(result)
    reference = splits["train"][0].metadata
    urdf = Path(get_package_share_directory(URDF_PACKAGE)) / URDF_RELATIVE_PATH
    gravity_model = {
        "urdf_package": URDF_PACKAGE,
        "urdf_relative_path": URDF_RELATIVE_PATH,
        "urdf_sha256": file_sha256(urdf),
        "frame": physical.FT_FRAME,
        "joint_names": [f"right_joint_{index}" for index in range(1, 7)],
        "mass_kg": payload["mass_kg"],
        "com_sensor_m": payload["com_sensor_m"],
    }
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "approved": False,
        "approval_reason": "development validation max exceeds 1 N; held-out not used",
        "sample_hz": SAMPLE_HZ,
        "base_feature_dim": BASE_FEATURE_DIM,
        "base_feature_contract": "q_rad[6],dq_rad_s[6],causal_qdd_rad_s2[6]",
        "forbidden_runtime_features": [
            "measured_wrench",
            "measured_joint_torque",
            "raw_joint_torque",
            "task_error",
        ],
        "target_contract": "physical_ft_sensor_frame_[Fx,Fy,Fz,Mx,My,Mz]",
        "ablation": PHYSICAL_RIDGE_ABLATION,
        "feature_mode": "short_multiscale",
        "feature_contract": (
            "sin(q),cos(q),current_dq,mean_dq_8_16_32,mean_qdd_8_16_32"
        ),
        "history": 32,
        "architecture": "ridge",
        "prediction_contract": PHYSICAL_RIDGE_CONTRACT,
        "zero_pose_deg": reference["zero_pose_deg"],
        "ft_frame": reference["ft_frame"],
        "observer_input_frame": reference["observer_input_frame"],
        "payload_id": reference["payload_id"],
        "controller_config_hash": reference["controller_config_hash"],
        "gravity_model": gravity_model,
        "physical_identification": payload,
        "regularization": 1.0,
        "selection_seed": sweep.SEED,
        "validation": result["metrics"],
        "validation_by_group": result["by_group"],
        "source_sha256": source_hashes(),
        "scope": "train13 fit and development validation3 only; held-out/test not loaded",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{output.name}.", dir=output.parent
    ) as temporary:
        bundle = Path(temporary)
        model_path = save_bundle(model, metadata, bundle)
        predictor = BundlePredictor(model_path, require_approved=False)

        runtime_prediction = runtime_predictions(predictor, splits["validation"])
        physical_prediction = np.concatenate(
            [
                physical.physical_wrench(deltas[session.path][31:], payload)
                for session in splits["validation"]
            ]
        )
        offline_prediction = result["prediction"] + physical_prediction
        targets = np.concatenate(
            [session.wrench[31:] for session in splits["validation"]]
        )
        parity_max = float(
            np.max(np.abs(runtime_prediction - offline_prediction))
        )
        if parity_max > PARITY_LIMIT_N:
            raise RuntimeError(
                f"runtime/offline prediction mismatch: {parity_max}"
            )
        runtime_validation = error_metrics(targets, runtime_prediction)
        benchmark = benchmark_predictor(
            predictor, splits["validation"], args.benchmark_calls
        )

        metadata.update(
            {
                "model_sha256": file_sha256(model_path),
                "runtime_validation": runtime_validation,
                "runtime_offline_parity_max_abs": parity_max,
                "runtime_offline_parity_limit": PARITY_LIMIT_N,
                "runtime_benchmark": benchmark,
            }
        )
        (bundle / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        manifest = sweep.training.session_manifest(splits)
        report = {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "source_commit": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], text=True
            ).strip(),
            "script_sha256": hashlib.sha256(
                Path(__file__).read_bytes()
            ).hexdigest(),
            "source_sha256": metadata["source_sha256"],
            "model_sha256": metadata["model_sha256"],
            "approved": False,
            "scope": metadata["scope"],
            "manifest": manifest,
            "validation": runtime_validation,
            "runtime_offline_parity_max_abs": parity_max,
            "runtime_offline_parity_limit": PARITY_LIMIT_N,
            "runtime_benchmark": benchmark,
            "warning": (
                "diagnostic runtime-compatible bundle; observer must reject it"
            ),
        }
        (bundle / "split_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (bundle / "bundle_benchmark_report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        bundle.rename(output)
    print(
        f"[DONE] validation_max={runtime_validation['force_norm_max_n']:.4f} N "
        f"p99={benchmark['p99_ms']:.4f} ms max={benchmark['max_ms']:.4f} ms "
        f"runtime_pass={benchmark['passed']} output={output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
