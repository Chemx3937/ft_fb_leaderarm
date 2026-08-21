#!/usr/bin/env python3
"""Explain the sustained EMA-5 residual in protected validation div02."""

import argparse
import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

import ft_fb_leaderarm.train_ablation as training
from ft_fb_leaderarm.contract import SAMPLE_HZ
import run_causal_residual_filter_validation as filtering
import run_physical_residual_validation as physical
import run_targeted6_validation_ablation as targeted


DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_div02_residual_diagnostic_v3_20260821"
)
DIV02 = "tare_20260819_div02"
AXES = ("Fx", "Fy", "Fz")


def distribution(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "samples": int(len(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95.0)),
        "max": float(np.max(values)),
    }


def fit_payload_delta_mass(residual, delta_g, mask=None):
    residual = np.asarray(residual, dtype=np.float64)
    delta_g = np.asarray(delta_g, dtype=np.float64)
    if mask is None:
        mask = np.ones(len(residual), dtype=bool)
    denominator = float(np.sum(delta_g[mask] ** 2))
    if denominator <= 1.0e-12:
        raise RuntimeError("gravity delta does not excite a payload-mass diagnostic")
    return float(np.sum(delta_g[mask] * residual[mask, :3]) / denominator)


def feature_coverage(train_sessions, validation_session, history):
    train_x, _ = training.build_matrix(
        train_sessions,
        "smoothed_dynamic",
        history,
        max_windows_per_session=5000,
        seed=8,
    )
    ends = np.arange(history - 1, len(validation_session.features))
    offsets = np.arange(history - 1, -1, -1)
    validation_x = targeted.diagnostic_projection(
        validation_session.features[ends[:, None] - offsets[None, :]],
        "smoothed_dynamic",
    )
    mean = train_x.mean(axis=0)
    std = np.maximum(train_x.std(axis=0), 1.0e-6)
    train_x = (train_x - mean) / std
    validation_x = (validation_x - mean) / std
    distances = {}
    for name, columns in {
        "full_24d": slice(None),
        "posture_sin_cos_12d": slice(0, 12),
        "velocity_6d": slice(12, 18),
        "smoothed_acceleration_6d": slice(18, 24),
    }.items():
        tree = cKDTree(train_x[:, columns])
        distances[name] = tree.query(validation_x[:, columns], workers=-1)[0]
    return distances, len(train_x)


def event_rows(mask, force, features, sync_error_ms, distances, task_error):
    rows = []
    for start, stop in targeted.contiguous_runs(mask, minimum_samples=1):
        event_force = force[start:stop, :3]
        force_norm = np.linalg.norm(event_force, axis=1)
        speed = np.linalg.norm(features[start:stop, 6:12], axis=1)
        acceleration = np.linalg.norm(features[start:stop, 12:18], axis=1)
        median_force = np.median(event_force, axis=0)
        q = features[start:stop, :6]
        rows.append(
            {
                "start_sample": start,
                "stop_sample_exclusive": stop,
                "start_s": start / SAMPLE_HZ,
                "end_s": (stop - 1) / SAMPLE_HZ,
                "samples": stop - start,
                "span_ms": max(0, stop - start - 1) / SAMPLE_HZ * 1000.0,
                "force_norm_n": distribution(force_norm),
                "force_median_n": median_force.tolist(),
                "dominant_median_axis": AXES[int(np.argmax(np.abs(median_force)))],
                "speed_norm_rad_s": distribution(speed),
                "acceleration_norm_rad_s2": distribution(acceleration),
                "absolute_sync_error_ms": distribution(
                    np.abs(sync_error_ms[start:stop])
                ),
                "training_feature_distance": {
                    name: distribution(distance[start:stop])
                    for name, distance in distances.items()
                },
                "max_joint_range_deg": float(np.max(np.rad2deg(np.ptp(q, axis=0)))),
                "joint_range_deg": np.rad2deg(np.ptp(q, axis=0)).tolist(),
                "task_translation_error_mm": distribution(
                    np.linalg.norm(task_error[start:stop, :3], axis=1)
                ),
                "task_rotation_error_deg": distribution(
                    np.linalg.norm(task_error[start:stop, 3:], axis=1)
                ),
            }
        )
    return sorted(rows, key=lambda row: (-row["samples"], row["start_sample"]))


def context_comparison(mask, features, sync_error_ms, distance, task_error):
    values = {
        "speed_norm_rad_s": np.linalg.norm(features[:, 6:12], axis=1),
        "acceleration_norm_rad_s2": np.linalg.norm(features[:, 12:18], axis=1),
        "absolute_sync_error_ms": np.abs(sync_error_ms),
        "training_feature_distance": distance,
        "task_translation_error_mm": np.linalg.norm(task_error[:, :3], axis=1),
        "task_rotation_error_deg": np.linalg.norm(task_error[:, 3:], axis=1),
    }
    return {
        name: {"over_1n": distribution(value[mask]), "at_most_1n": distribution(value[~mask])}
        for name, value in values.items()
    }


def oracle_diagnostics(residual, delta_g):
    sample_time = np.arange(len(residual), dtype=np.float64) / SAMPLE_HZ
    centered_time = sample_time - np.mean(sample_time)
    design = np.column_stack((np.ones(len(residual)), centered_time))
    trend = design @ np.linalg.lstsq(design, residual, rcond=None)[0]
    offset = np.median(residual, axis=0)
    delta_mass = fit_payload_delta_mass(residual, delta_g)
    payload = np.zeros_like(residual)
    payload[:, :3] = delta_mass * delta_g
    candidates = {
        "none": residual,
        "group_constant_offset_oracle": residual - offset,
        "time_linear_offset_and_drift_oracle": residual - trend,
        "payload_delta_mass_oracle": residual - payload,
        "payload_delta_mass_plus_constant_offset_oracle": residual - payload - np.median(
            residual - payload, axis=0
        ),
    }
    return {
        "warning": "validation-fitted diagnostic oracles; forbidden for model selection",
        "fitted_payload_delta_mass_kg": delta_mass,
        "fitted_group_offset_wrench": offset.tolist(),
        "ema5_metrics": {
            name: filtering.force_metrics(filtering.apply_filter("ema5hz", values))
            for name, values in candidates.items()
        },
    }


def coverage_summary(distances, force_norm, over_1n):
    return {
        name: {
            "all_div02": distribution(distance),
            "over_1n": distribution(distance[over_1n]),
            "at_most_1n": distribution(distance[~over_1n]),
            "distance_force_error_pearson": float(
                np.corrcoef(distance, force_norm)[0, 1]
            ),
            "over_1n_median_percentile_within_div02": float(
                100.0 * np.mean(distance <= np.median(distance[over_1n]))
            ),
        }
        for name, distance in distances.items()
    }


def self_check():
    delta_g = np.arange(30, dtype=np.float64).reshape(10, 3) / 10.0
    residual = np.zeros((10, 6), dtype=np.float64)
    residual[:, :3] = 0.25 * delta_g
    assert np.isclose(fit_payload_delta_mass(residual, delta_g), 0.25)
    assert np.isclose(
        oracle_diagnostics(residual, delta_g)["fitted_payload_delta_mass_kg"],
        0.25,
    )
    rows = event_rows(
        np.asarray([False, True, True, False]),
        np.ones((4, 6)),
        np.zeros((4, 18)),
        np.zeros(4),
        {"full_24d": np.zeros(4)},
        np.zeros((4, 6)),
    )
    assert rows[0]["samples"] == 2 and rows[0]["start_sample"] == 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    self_check()
    if args.self_check:
        print("div02 residual diagnostic self-check: PASS")
        return

    output = Path(args.output_dir).expanduser().resolve()
    started = time.monotonic()
    training.project_feature_windows = targeted.diagnostic_projection
    training.ABLATIONS = {**training.ABLATIONS, **physical.CANDIDATES}
    splits = targeted.load_fixed_splits()
    if set(splits) != {"train", "validation"}:
        raise RuntimeError("offline diagnostic refuses any held-out test split")
    deltas = physical.gravity_deltas(splits)
    payload = physical.identify_payload(splits["train"], deltas)
    residual_splits = physical.residualize(splits, deltas, payload)
    selected_name = "physical_residual_smoothed_dynamic_mlp"
    trained = training.train_candidate(
        selected_name, residual_splits, seed=8, **targeted.SETTINGS
    )
    session = next(row for row in residual_splits["validation"] if row.group == DIV02)
    models = {selected_name: (trained["model"], trained["mode"], trained["history"])}
    aligned_start, target, prediction = targeted.aligned_prediction(models, session)
    residual = target - prediction
    ema5 = filtering.apply_filter("ema5hz", residual)
    ema5_norm = np.linalg.norm(ema5[:, :3], axis=1)
    over_1n = ema5_norm > 1.0
    features = session.features[aligned_start:]
    delta_g = deltas[session.path][aligned_start:]
    distances, coverage_train_samples = feature_coverage(
        residual_splits["train"], session, trained["history"]
    )
    distance = distances["full_24d"]
    with np.load(session.path, allow_pickle=False) as archive:
        sync_error_ms = np.asarray(archive["sync_error_ms"], dtype=np.float64)[
            aligned_start:
        ]
        task_error = np.asarray(archive["task_error"], dtype=np.float64)[aligned_start:]

    events = event_rows(
        over_1n, ema5, features, sync_error_ms, distances, task_error
    )
    force_norm_by_filter = {
        name: filtering.force_metrics(filtering.apply_filter(name, residual))
        for name in ("raw", "ema20hz", "ema10hz", "ema5hz", "ema2p5hz")
    }
    flattened_cosine = float(
        np.sum(residual[:, :3] * delta_g)
        / (np.linalg.norm(residual[:, :3]) * np.linalg.norm(delta_g))
    )
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "offline train13 + protected validation div02 only; held-out not loaded",
        "approved": False,
        "runtime_or_acceptance_changed": False,
        "diagnostic_session": DIV02,
        "model": selected_name,
        "sample_hz": SAMPLE_HZ,
        "task_error_unit_contract": "translation mm; rotation deg",
        "manifest": training.session_manifest(splits),
        "physical_identification": payload,
        "force_metrics_by_causal_filter": force_norm_by_filter,
        "ema5_over_1n": {
            "samples": int(np.count_nonzero(over_1n)),
            "fraction": float(np.mean(over_1n)),
            "events": len(events),
            "longest_events": events[:10],
        },
        "over_1n_context_comparison": context_comparison(
            over_1n, features, sync_error_ms, distance, task_error
        ),
        "training_feature_coverage": {
            "contract": (
                "nearest of deterministic up-to-5000 windows per train session; "
                "train-standardized 24D smoothed-dynamic features"
            ),
            "train_samples": coverage_train_samples,
            "by_feature_component": coverage_summary(
                distances, ema5_norm, over_1n
            ),
        },
        "gravity_alignment": {
            "flattened_force_residual_delta_g_cosine": flattened_cosine,
            "fitted_payload_delta_mass_kg": fit_payload_delta_mass(residual, delta_g),
        },
        "validation_fitted_oracle_diagnostics": oracle_diagnostics(residual, delta_g),
        "elapsed_s": time.monotonic() - started,
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "split_manifest.json").write_text(
        json.dumps(report["manifest"], indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output / "div02_residual_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"[DONE] output={output}", flush=True)


if __name__ == "__main__":
    main()
