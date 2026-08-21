#!/usr/bin/env python3
"""Validation-only targeted6 ablation with the protected fixed split."""

import argparse
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

import ft_fb_leaderarm.train_ablation as training
from ft_fb_leaderarm.contract import error_metrics, project_feature_windows


OLD = Path("/home/vision/.ros/ft_fb_leaderarm/datasets/right_final13_20260819")
NEW = Path("/home/vision/.ros/ft_fb_leaderarm/datasets/right_targeted6_20260821")
DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_targeted6_diagnostic_20260821"
)
EPISODES = {
    "train": (
        ("tare_20260819_02", OLD / "right_free_space_20260819_034917.npz"),
        ("tare_20260819_03", OLD / "right_free_space_20260819_040748.npz"),
        ("tare_20260819_04", OLD / "right_free_space_20260819_041307.npz"),
        ("tare_20260819_div03", OLD / "right_free_space_20260819_045441.npz"),
        ("tare_20260819_div04", OLD / "right_free_space_20260819_050000.npz"),
        ("tare_20260819_div06", OLD / "right_free_space_20260819_050941.npz"),
        ("tare_20260819_div09", OLD / "right_free_space_20260819_052432.npz"),
        ("tare_20260821_target01", NEW / "right_free_space_20260821_170754.npz"),
        ("tare_20260821_target02", NEW / "right_free_space_20260821_171634.npz"),
        (
            "tare_20260821_target03_retry01",
            NEW / "right_free_space_20260821_174146.npz",
        ),
        (
            "tare_20260821_target04_retry01",
            NEW / "right_free_space_20260821_175737.npz",
        ),
        ("tare_20260821_target05", NEW / "right_free_space_20260821_180730.npz"),
        ("tare_20260821_target06", NEW / "right_free_space_20260821_181611.npz"),
    ),
    "validation": (
        ("tare_20260819_div02", OLD / "right_free_space_20260819_044927.npz"),
        ("tare_20260819_div05", OLD / "right_free_space_20260819_050511.npz"),
        ("tare_20260819_div07", OLD / "right_free_space_20260819_051410.npz"),
    ),
}
SETTINGS = {
    "epochs": 60,
    "batch_size": 1024,
    "learning_rate": 1.0e-3,
    "max_windows_per_session": 20_000,
}
TASK_GROUPS = (
    "tare_20260819_02",
    "tare_20260819_03",
    "tare_20260819_04",
)


original_projection = project_feature_windows


def diagnostic_projection(windows, mode):
    if mode not in ("smoothed_dynamic", "lag2_smoothed_dynamic"):
        return original_projection(windows, mode)
    array = np.asarray(windows, dtype=np.float32)
    if array.ndim != 3 or array.shape[2] != 18:
        raise ValueError("feature windows must have shape [N,H,18]")
    lagged = mode == "lag2_smoothed_dynamic"
    if lagged and array.shape[1] < 10:
        raise ValueError("lag2 smoothed windows require at least 10 samples")
    current = -3 if lagged else -1
    qdd = array[:, -10:-2, 12:18] if lagged else array[:, :, 12:18]
    q = array[:, current, :6]
    return np.concatenate(
        (
            np.sin(q),
            np.cos(q),
            array[:, current, 6:12],
            np.mean(qdd, axis=1),
        ),
        axis=1,
    )


def load_fixed_splits():
    splits = {
        role: [training.load_session(path) for _, path in entries]
        for role, entries in EPISODES.items()
    }
    for role, entries in EPISODES.items():
        actual = [session.group for session in splits[role]]
        expected = [zero_set_id for zero_set_id, _ in entries]
        if actual != expected:
            raise RuntimeError(f"{role} zero-set IDs do not match the fixed manifest")
    all_sessions = splits["train"] + splits["validation"]
    if len({session.group for session in all_sessions}) != len(all_sessions):
        raise RuntimeError("fixed manifest contains duplicate zero-set IDs")
    reference = all_sessions[0].metadata
    for session in all_sessions[1:]:
        for key in (
            "ft_frame",
            "observer_input_frame",
            "payload_id",
            "controller_config_hash",
        ):
            if session.metadata.get(key) != reference.get(key):
                raise RuntimeError(f"{session.path}: {key} contract differs")
        if not np.allclose(
            session.metadata.get("zero_pose_deg", []),
            reference.get("zero_pose_deg", []),
            rtol=0.0,
            atol=1.0e-9,
        ):
            raise RuntimeError(f"{session.path}: zero_pose_deg contract differs")
    return splits


def predict_session(model, session, mode, history):
    end_indices = np.arange(history - 1, len(session.features))
    offsets = np.arange(history - 1, -1, -1)
    predictions = []
    for start in range(0, len(end_indices), 8192):
        ends = end_indices[start : start + 8192]
        windows = session.features[ends[:, None] - offsets[None, :]]
        projected = diagnostic_projection(windows, mode)
        predictions.append(training.predict_batches(model, projected))
    return np.concatenate(predictions)


def aligned_prediction(models, session):
    max_history = max(history for _, _, history in models.values())
    target = session.wrench[max_history - 1 :]
    predictions = []
    for model, mode, history in models.values():
        prediction = predict_session(model, session, mode, history)
        predictions.append(prediction[max_history - history :])
    return max_history - 1, target, np.mean(predictions, axis=0)


def evaluate_models(models, sessions):
    all_targets, all_predictions = [], []
    by_group = {}
    for session in sessions:
        _, target, prediction = aligned_prediction(models, session)
        by_group[session.group] = error_metrics(target, prediction)
        all_targets.append(target)
        all_predictions.append(prediction)
    return {
        "validation": error_metrics(
            np.concatenate(all_targets), np.concatenate(all_predictions)
        ),
        "validation_by_group": by_group,
    }


def shifted(values, lag):
    if lag > 0:
        return values[lag:]
    if lag < 0:
        return values[:lag]
    return values


def contiguous_runs(mask, minimum_samples):
    padded = np.pad(np.asarray(mask, dtype=np.int8), (1, 1))
    edges = np.flatnonzero(np.diff(padded))
    return [
        (int(start), int(stop))
        for start, stop in edges.reshape(-1, 2)
        if stop - start >= minimum_samples
    ]


def lag_summary(models, sessions):
    pairs = [aligned_prediction(models, session)[1:] for session in sessions]
    scan = {}
    for lag in range(-16, 17):
        targets = [shifted(target, lag) for target, _ in pairs]
        predictions = [shifted(prediction, -lag) for _, prediction in pairs]
        scan[lag] = error_metrics(
            np.concatenate(targets), np.concatenate(predictions)
        )
    best_lag = min(scan, key=lambda lag: scan[lag]["force_norm_rmse_n"])
    return {
        "best_force_rmse_lag_samples": best_lag,
        "zero_lag": scan[0],
        "best_lag": scan[best_lag],
    }


def residual_diagnostics(models, sessions):
    rows = []
    for session in sessions:
        start, target, prediction = aligned_prediction(models, session)
        with np.load(session.path, allow_pickle=False) as archive:
            sync_error_ms = np.asarray(archive["sync_error_ms"], dtype=np.float64)[
                start:
            ]
        rows.append(
            {
                "session": session,
                "start": start,
                "target": target,
                "prediction": prediction,
                "sync_error_ms": sync_error_ms,
                "speed": np.linalg.norm(session.features[start:, 6:12], axis=1),
                "acceleration": np.linalg.norm(
                    session.features[start:, 12:18], axis=1
                ),
            }
        )

    lag_scan = {}
    for lag in range(-16, 17):
        targets = [shifted(row["target"], lag) for row in rows]
        predictions = [shifted(row["prediction"], -lag) for row in rows]
        lag_scan[str(lag)] = error_metrics(
            np.concatenate(targets), np.concatenate(predictions)
        )
    best_lag = min(
        lag_scan,
        key=lambda lag: lag_scan[lag]["force_norm_rmse_n"],
    )

    offset_targets, offset_predictions = [], []
    offsets = {}
    for row in rows:
        residual = row["target"] - row["prediction"]
        offset = np.median(residual, axis=0)
        offsets[row["session"].group] = offset.tolist()
        offset_targets.append(row["target"])
        offset_predictions.append(row["prediction"] + offset)

    target = np.concatenate([row["target"] for row in rows])
    prediction = np.concatenate([row["prediction"] for row in rows])
    speed = np.concatenate([row["speed"] for row in rows])
    acceleration = np.concatenate([row["acceleration"] for row in rows])
    sync_error_ms = np.concatenate([row["sync_error_ms"] for row in rows])
    force_error = np.linalg.norm((target - prediction)[:, :3], axis=1)
    correlations = {
        "speed_norm": float(np.corrcoef(force_error, speed)[0, 1]),
        "acceleration_norm": float(
            np.corrcoef(force_error, acceleration)[0, 1]
        ),
        "absolute_sync_error_ms": float(
            np.corrcoef(force_error, np.abs(sync_error_ms))[0, 1]
        ),
    }
    motion_bins = {}
    for name, values in (("speed", speed), ("acceleration", acceleration)):
        metrics = []
        for indices in np.array_split(np.argsort(values), 4):
            metrics.append(
                {
                    "lower": float(np.min(values[indices])),
                    "upper": float(np.max(values[indices])),
                    "metrics": error_metrics(
                        target[indices], prediction[indices]
                    ),
                }
            )
        motion_bins[name] = metrics

    stationary_runs = []
    for row in rows:
        stationary = row["speed"] <= 0.02
        for start, stop in contiguous_runs(stationary, minimum_samples=263):
            q = row["session"].features[
                row["start"] + start : row["start"] + stop, :6
            ]
            q_range_deg = np.rad2deg(np.ptp(q, axis=0))
            if float(np.max(q_range_deg)) > 0.2:
                continue
            wrench = row["target"][start:stop]
            centered = wrench[:, :3] - np.median(wrench[:, :3], axis=0)
            force_norm = np.linalg.norm(centered, axis=1)
            stationary_runs.append(
                {
                    "zero_set_id": row["session"].group,
                    "samples": stop - start,
                    "duration_s": (stop - start) / 262.5,
                    "max_joint_range_deg": float(np.max(q_range_deg)),
                    "force_axis_std_n": np.std(wrench[:, :3], axis=0).tolist(),
                    "force_deviation_p95_n": float(np.percentile(force_norm, 95)),
                    "force_deviation_max_n": float(np.max(force_norm)),
                }
            )
    return {
        "lag_convention": "positive lag compares later target with earlier prediction",
        "lag_scan_samples": lag_scan,
        "best_force_rmse_lag_samples": int(best_lag),
        "group_constant_offset": offsets,
        "group_offset_oracle": error_metrics(
            np.concatenate(offset_targets), np.concatenate(offset_predictions)
        ),
        "force_error_correlations": correlations,
        "motion_quartiles": motion_bins,
        "stationary_runs_at_least_1s": stationary_runs,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    return parser.parse_args()


def main():
    args = parse_args()
    output = Path(args.output_dir).expanduser().resolve()
    probe = np.zeros((1, 8, 18), dtype=np.float32)
    probe[0, :, 12:] = np.arange(8, dtype=np.float32)[:, None]
    assert np.allclose(diagnostic_projection(probe, "smoothed_dynamic")[0, 18:], 3.5)
    lagged_probe = np.zeros((1, 10, 18), dtype=np.float32)
    lagged_probe[0, :, 12:] = np.arange(10, dtype=np.float32)[:, None]
    assert np.allclose(
        diagnostic_projection(lagged_probe, "lag2_smoothed_dynamic")[0, 18:],
        3.5,
    )

    training.project_feature_windows = diagnostic_projection
    training.ABLATIONS = {
        **training.ABLATIONS,
        "smoothed_dynamic_mlp": ("smoothed_dynamic", 8, (128, 128), "mlp"),
        "lag2_smoothed_dynamic_mlp": (
            "lag2_smoothed_dynamic",
            10,
            (128, 128),
            "mlp",
        ),
        "short_history_mlp": ("history", 8, (128, 128), "mlp"),
    }
    splits = load_fixed_splits()
    manifest = training.session_manifest(splits)

    started = time.monotonic()
    trained = {}
    for name, seed in (
        ("static_linear", 7),
        ("dynamic_mlp", 8),
        ("smoothed_dynamic_mlp", 8),
        ("lag2_smoothed_dynamic_mlp", 8),
        ("short_history_mlp", 9),
    ):
        print(f"[TARGETED6] training {name}", flush=True)
        result = training.train_candidate(name, splits, seed=seed, **SETTINGS)
        trained[name] = result
        metrics = result["validation"]
        print(
            f"[TARGETED6] {name}: max={metrics['force_norm_max_n']:.4f} "
            f"p95={metrics['force_norm_p95_n']:.4f} "
            f"rmse={metrics['force_norm_rmse_n']:.4f}",
            flush=True,
        )

    candidates = {
        name: {
            "best_epoch": row["best_epoch"],
            "feature_mode": row["mode"],
            "history": row["history"],
            "train_samples": row["train_samples"],
            "validation": row["validation"],
            "validation_by_group": {
                session.group: training.evaluate_all_windows(
                    row["model"], [session], row["mode"], row["history"]
                )
                for session in splits["validation"]
            },
        }
        for name, row in trained.items()
    }
    task_sessions = [
        session for session in splits["train"] if session.group in TASK_GROUPS
    ]
    if [session.group for session in task_sessions] != list(TASK_GROUPS):
        raise RuntimeError("task evaluation groups do not match the fixed contract")
    task_evaluation = {
        name: {
            "metrics": training.evaluate_all_windows(
                row["model"], task_sessions, row["mode"], row["history"]
            ),
            "by_group": {
                session.group: training.evaluate_all_windows(
                    row["model"], [session], row["mode"], row["history"]
                )
                for session in task_sessions
            },
        }
        for name, row in trained.items()
    }
    ensemble_models = {
        name: (
            trained[name]["model"],
            trained[name]["mode"],
            trained[name]["history"],
        )
        for name in ("dynamic_mlp", "short_history_mlp")
    }
    ensemble = evaluate_models(
        ensemble_models,
        splits["validation"],
    )
    ensemble_task = evaluate_models(ensemble_models, task_sessions)
    task_evaluation["dynamic_mlp+short_history_mlp"] = {
        "metrics": ensemble_task["validation"],
        "by_group": ensemble_task["validation_by_group"],
    }
    ranked = {
        **{name: row["validation"] for name, row in candidates.items()},
        "dynamic_mlp+short_history_mlp": ensemble["validation"],
    }
    selected_name, selected_metrics = min(
        ranked.items(),
        key=lambda item: (
            item[1]["force_norm_max_n"],
            item[1]["force_norm_p95_n"],
            item[1]["force_norm_rmse_n"],
        ),
    )
    if selected_name == "dynamic_mlp+short_history_mlp":
        selected_models = ensemble_models
    else:
        selected = trained[selected_name]
        selected_models = {
            selected_name: (
                selected["model"],
                selected["mode"],
                selected["history"],
            )
        }
    diagnostics = residual_diagnostics(selected_models, splits["validation"])
    candidate_lag_summary = {
        name: lag_summary(
            {
                name: (row["model"], row["mode"], row["history"]),
            },
            splits["validation"],
        )
        for name, row in trained.items()
    }
    candidate_lag_summary["dynamic_mlp+short_history_mlp"] = lag_summary(
        ensemble_models, splits["validation"]
    )
    maximum = selected_metrics["force_norm_max_n"]
    if maximum <= 1.0:
        next_action = "freeze_method_and_collect_new_held_out_test"
    elif maximum < 3.0:
        next_action = "review_group_residuals_before_one_more_targeted_batch"
    else:
        next_action = "stop_collection_and_separate_sync_payload_gravity_target_noise"
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "scope": "validation-only; past held-out test was not loaded",
        "approved": False,
        "selection_uses": "protected validation only",
        "settings": SETTINGS,
        "manifest": manifest,
        "candidates": candidates,
        "ensemble": {
            "members": ["dynamic_mlp", "short_history_mlp"],
            **ensemble,
        },
        "task_evaluation_in_sample": task_evaluation,
        "selected_method": selected_name,
        "selected_validation": selected_metrics,
        "candidate_lag_summary": candidate_lag_summary,
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
