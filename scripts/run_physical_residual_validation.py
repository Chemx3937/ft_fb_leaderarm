#!/usr/bin/env python3
"""Fit train-only payload gravity, then validate dynamic residual MLPs."""

import argparse
import hashlib
import json
import subprocess
import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pinocchio as pin

import ft_fb_leaderarm.train_ablation as training
from ft_fb_leaderarm.contract import error_metrics
import run_targeted6_validation_ablation as targeted


FOLLOWER_URDF = Path(
    "/home/vision/dualarm_ws/src/aidin_dsr_dualarm/"
    "aidin_dsr_dualarm_description/urdf/aidin_dsr_dualarm_aligned_hand.urdf"
)
FT_FRAME = "right_link_6"
DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_physical_residual_diagnostic_v4_20260821"
)
CANDIDATES = {
    "physical_residual_dynamic_mlp": ("dynamic", 1, (128, 128), "mlp"),
    "physical_residual_smoothed_dynamic_mlp": (
        "smoothed_dynamic",
        8,
        (128, 128),
        "mlp",
    ),
    "physical_residual_lag2_smoothed_dynamic_mlp": (
        "lag2_smoothed_dynamic",
        10,
        (128, 128),
        "mlp",
    ),
}


def moment_design(force):
    """Return A such that A @ r is r cross force for each sample."""
    force = np.asarray(force, dtype=np.float64)
    if force.ndim != 2 or force.shape[1] != 3:
        raise ValueError("force must have shape [N,3]")
    fx, fy, fz = force.T
    matrix = np.zeros((len(force), 3, 3), dtype=np.float64)
    matrix[:, 0, 1] = fz
    matrix[:, 0, 2] = -fy
    matrix[:, 1, 0] = -fz
    matrix[:, 1, 2] = fx
    matrix[:, 2, 0] = fy
    matrix[:, 2, 1] = -fx
    return matrix.reshape(-1, 3)


def gravity_deltas(splits):
    model = pin.buildModelFromUrdf(str(FOLLOWER_URDF))
    data = model.createData()
    frame_id = model.getFrameId(FT_FRAME)
    if frame_id >= len(model.frames):
        raise RuntimeError(f"missing follower frame: {FT_FRAME}")
    joint_indices = [
        model.joints[model.getJointId(f"right_joint_{index}")].idx_q
        for index in range(1, 7)
    ]
    q_full = pin.neutral(model)

    def sensor_gravity(q_rows):
        values = np.empty((len(q_rows), 3), dtype=np.float64)
        for index, q in enumerate(q_rows):
            q_full[joint_indices] = q
            pin.forwardKinematics(model, data, q_full)
            pin.updateFramePlacements(model, data)
            values[index] = (
                data.oMf[frame_id].rotation.T @ model.gravity.linear
            )
        return values

    sessions = splits["train"] + splits["validation"]
    zero_pose = np.deg2rad(sessions[0].metadata["zero_pose_deg"])
    gravity_at_zero = sensor_gravity(zero_pose[None, :])[0]
    return {
        session.path: sensor_gravity(session.features[:, :6]) - gravity_at_zero
        for session in sessions
    }


def identify_payload(train_sessions, deltas):
    acceleration = np.concatenate(
        [np.linalg.norm(session.features[:, 12:18], axis=1) for session in train_sessions]
    )
    threshold = float(np.percentile(acceleration, 25.0))
    force = np.concatenate([session.wrench[:, :3] for session in train_sessions])
    moment = np.concatenate([session.wrench[:, 3:] for session in train_sessions])
    delta = np.concatenate([deltas[session.path] for session in train_sessions])
    mask = acceleration <= threshold
    delta_fit = delta[mask]
    force_fit = force[mask]
    denominator = float(np.sum(delta_fit * delta_fit))
    if denominator <= 1.0e-12:
        raise RuntimeError("training poses do not excite payload gravity")
    mass = float(np.sum(delta_fit * force_fit) / denominator)
    if not np.isfinite(mass) or mass <= 0.0:
        raise RuntimeError("identified payload mass must be positive")
    gravity_force = mass * delta_fit
    design = moment_design(gravity_force)
    com, _, rank, singular_values = np.linalg.lstsq(
        design, moment[mask].reshape(-1), rcond=None
    )
    if rank != 3 or not np.isfinite(com).all():
        raise RuntimeError("payload CoM fit is rank deficient")
    return {
        "method": "train acceleration lower quartile; F=m*delta_g; M=r_cross_F",
        "fit_samples": int(np.count_nonzero(mask)),
        "acceleration_q25_rad_s2": threshold,
        "mass_kg": mass,
        "com_sensor_m": com.tolist(),
        "moment_design_condition": float(singular_values[0] / singular_values[-1]),
    }


def physical_wrench(delta, payload):
    force = float(payload["mass_kg"]) * np.asarray(delta, dtype=np.float64)
    com = np.asarray(payload["com_sensor_m"], dtype=np.float64)
    moment = np.cross(np.broadcast_to(com, force.shape), force)
    return np.column_stack((force, moment)).astype(np.float32)


def physical_evaluation(sessions, deltas, payload):
    targets, predictions = [], []
    by_group = {}
    for session in sessions:
        prediction = physical_wrench(deltas[session.path], payload)
        by_group[session.group] = error_metrics(session.wrench, prediction)
        targets.append(session.wrench)
        predictions.append(prediction)
    return {
        "metrics": error_metrics(np.concatenate(targets), np.concatenate(predictions)),
        "by_group": by_group,
    }


def residualize(splits, deltas, payload):
    return {
        role: [
            replace(
                session,
                wrench=(
                    session.wrench
                    - physical_wrench(deltas[session.path], payload)
                ).astype(np.float32),
            )
            for session in sessions
        ]
        for role, sessions in splits.items()
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    return parser.parse_args()


def main():
    args = parse_args()
    output = Path(args.output_dir).expanduser().resolve()
    probe_force = np.asarray([[4.0, 5.0, 6.0]])
    probe_com = np.asarray([1.0, 2.0, 3.0])
    assert np.allclose(
        moment_design(probe_force) @ probe_com,
        np.cross(probe_com, probe_force[0]),
    )

    training.project_feature_windows = targeted.diagnostic_projection
    training.ABLATIONS = {**training.ABLATIONS, **CANDIDATES}
    splits = targeted.load_fixed_splits()
    manifest = training.session_manifest(splits)
    started = time.monotonic()
    deltas = gravity_deltas(splits)
    payload = identify_payload(splits["train"], deltas)
    physical = physical_evaluation(splits["validation"], deltas, payload)
    residual_splits = residualize(splits, deltas, payload)

    candidates = {}
    trained = {}
    for name, seed in (
        ("physical_residual_dynamic_mlp", 8),
        ("physical_residual_smoothed_dynamic_mlp", 8),
        ("physical_residual_lag2_smoothed_dynamic_mlp", 8),
    ):
        print(f"[PHYSICAL-RESIDUAL] training {name}", flush=True)
        result = training.train_candidate(
            name, residual_splits, seed=seed, **targeted.SETTINGS
        )
        trained[name] = result
        candidates[name] = {
            "best_epoch": result["best_epoch"],
            "feature_mode": result["mode"],
            "history": result["history"],
            "train_samples": result["train_samples"],
            "validation": result["validation"],
            "validation_by_group": {
                session.group: training.evaluate_all_windows(
                    result["model"], [session], result["mode"], result["history"]
                )
                for session in residual_splits["validation"]
            },
        }
        metrics = result["validation"]
        print(
            f"[PHYSICAL-RESIDUAL] {name}: "
            f"max={metrics['force_norm_max_n']:.4f} "
            f"p95={metrics['force_norm_p95_n']:.4f} "
            f"rmse={metrics['force_norm_rmse_n']:.4f}",
            flush=True,
        )

    task_sessions = [
        session
        for session in residual_splits["train"]
        if session.group in targeted.TASK_GROUPS
    ]
    if [session.group for session in task_sessions] != list(targeted.TASK_GROUPS):
        raise RuntimeError("task evaluation groups do not match the fixed contract")
    task_evaluation = {
        name: {
            "metrics": training.evaluate_all_windows(
                result["model"], task_sessions, result["mode"], result["history"]
            ),
            "by_group": {
                session.group: training.evaluate_all_windows(
                    result["model"], [session], result["mode"], result["history"]
                )
                for session in task_sessions
            },
        }
        for name, result in trained.items()
    }
    selected_name, selected = min(
        candidates.items(),
        key=lambda item: (
            item[1]["validation"]["force_norm_max_n"],
            item[1]["validation"]["force_norm_p95_n"],
            item[1]["validation"]["force_norm_rmse_n"],
        ),
    )
    maximum = selected["validation"]["force_norm_max_n"]
    selected_result = trained[selected_name]
    selected_models = {
        selected_name: (
            selected_result["model"],
            selected_result["mode"],
            selected_result["history"],
        )
    }
    diagnostics = targeted.residual_diagnostics(
        selected_models, residual_splits["validation"]
    )
    lag_summary = targeted.lag_summary(
        selected_models, residual_splits["validation"]
    )
    if maximum <= 1.0:
        next_action = "freeze_method_and_collect_new_held_out_test"
    elif maximum < 3.0:
        next_action = "review_residuals_without_reusing_past_test"
    else:
        next_action = "stop_collection_and_separate_cable_target_noise"
    script_path = Path(__file__).resolve()
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "script_sha256": hashlib.sha256(script_path.read_bytes()).hexdigest(),
        "scope": "train13 identification and protected validation3 only; past test not loaded",
        "approved": False,
        "selection_uses": "protected validation only",
        "prediction_contract": "physical payload gravity + learned dynamic residual",
        "settings": targeted.SETTINGS,
        "manifest": manifest,
        "physical_identification": payload,
        "physical_baseline_validation": physical,
        "candidates": candidates,
        "task_evaluation_in_sample": task_evaluation,
        "selected_method": selected_name,
        "selected_validation": selected["validation"],
        "selected_lag_summary": lag_summary,
        "residual_diagnostics": diagnostics,
        "validation_gate_pass": maximum <= 1.0,
        "next_action": next_action,
        "elapsed_s": time.monotonic() - started,
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "split_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "validation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"[DONE] selected={selected_name} max={maximum:.4f} "
        f"next={next_action} output={output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
