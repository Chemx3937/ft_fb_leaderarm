#!/usr/bin/env python3
"""Replay the current diagnostic wrench model over logistic-box IL episodes."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import tempfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import torch
import zarr

from ft_fb_leaderarm.contract import (
    FORCE_GROUP_P95_LIMIT_N,
    FORCE_HARD_MAX_LIMIT_N,
    FORCE_P99_LIMIT_N,
    SchmittContactDetector,
    error_metrics,
    project_feature_windows,
)
from ft_fb_leaderarm.model import BundlePredictor, file_sha256


DEFAULT_DATA = Path("/data/logistic_box_contact_observer")
DEFAULT_MODEL = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_ridge_short_multiscale_bundle_v3_20260822/model.ts"
)
DEFAULT_FREE_OUTPUT = Path(
    "/home/vision/dualarm_ws/src/ft_fb_leaderarm/document/experiment/"
    "free_space_wrench_model_validation"
)
DEFAULT_CONTACT_OUTPUT = Path(
    "/home/vision/dualarm_ws/src/ft_fb_leaderarm/document/experiment/"
    "contact_observer_validation"
)
FORCE_ON_N = 2.0
FORCE_OFF_N = 1.2
CONTACT_HOLD_S = 0.008
FREE_HOLD_S = 0.020
AXES = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
FIDELITY_LABELS = {
    "high_rate_observer_log": "262.5 Hz q,dq",
    "joint_30hz_interpolated": "30 Hz joint interpolation",
}
FREE_GUARD_MS = (0, 50, 100, 200, 500)
PRIMARY_FREE_GUARD_MS = 200


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--free-output", type=Path, default=DEFAULT_FREE_OUTPUT)
    parser.add_argument("--contact-output", type=Path, default=DEFAULT_CONTACT_OUTPUT)
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args(argv)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_array(episode, relative):
    path = episode / relative
    if not (path / ".zarray").is_file():
        raise RuntimeError(f"missing Zarr array: {path}")
    value = np.asarray(zarr.open(str(path), mode="r"))
    if not value.size or not np.isfinite(value).all():
        raise RuntimeError(f"empty or non-finite Zarr array: {path}")
    return value


def nearest_indices(reference, query):
    reference = np.asarray(reference, dtype=np.float64)
    query = np.asarray(query, dtype=np.float64)
    if reference.ndim != 1 or query.ndim != 1 or len(reference) < 2:
        raise ValueError("nearest lookup requires one-dimensional timestamps")
    if np.any(np.diff(reference) <= 0.0):
        raise ValueError("reference timestamps must increase")
    right = np.clip(np.searchsorted(reference, query), 0, len(reference) - 1)
    left = np.maximum(right - 1, 0)
    return np.where(
        np.abs(reference[left] - query) < np.abs(reference[right] - query),
        left,
        right,
    )


def binary_intervals(values):
    values = np.asarray(values, dtype=bool)
    edges = np.diff(np.r_[False, values, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def guarded_free_mask(reference, times, guard_ms):
    reference = np.asarray(reference, dtype=bool)
    times = np.asarray(times, dtype=np.float64)
    if reference.shape != times.shape:
        raise ValueError("reference and timestamps must have equal shapes")
    excluded = reference.copy()
    guard_s = float(guard_ms) / 1000.0
    for start, end in binary_intervals(reference):
        excluded |= (times >= times[start] - guard_s) & (
            times <= times[end - 1] + guard_s
        )
    return ~excluded


def safe_ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


def percentile_or_none(values, percentile):
    return float(np.percentile(values, percentile)) if values else None


def contact_metrics(reference, predicted, times):
    reference = np.asarray(reference, dtype=bool)
    predicted = np.asarray(predicted, dtype=bool)
    times = np.asarray(times, dtype=np.float64)
    if reference.shape != predicted.shape or reference.shape != times.shape:
        raise ValueError("contact arrays must have equal shapes")
    tp = int(np.sum(reference & predicted))
    fp = int(np.sum(~reference & predicted))
    fn = int(np.sum(reference & ~predicted))
    tn = int(np.sum(~reference & ~predicted))
    precision = safe_ratio(tp, tp + fp)
    recall = safe_ratio(tp, tp + fn)
    specificity = safe_ratio(tn, tn + fp)
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if precision is not None and recall is not None and precision + recall
        else None
    )
    reference_events = binary_intervals(reference)
    predicted_events = binary_intervals(predicted)
    onset_ms = []
    release_ms = []
    matched_reference = 0
    for event_index, (start, end) in enumerate(reference_events):
        hits = np.flatnonzero(predicted[start:end])
        if hits.size:
            matched_reference += 1
            onset_ms.append(1000.0 * (times[start + hits[0]] - times[start]))
        next_start = (
            reference_events[event_index + 1][0]
            if event_index + 1 < len(reference_events)
            else len(reference)
        )
        if end < len(reference):
            releases = np.flatnonzero(~predicted[end:next_start])
            if releases.size:
                release_ms.append(1000.0 * (times[end + releases[0]] - times[end]))
    matched_predicted = sum(
        bool(np.any(reference[start:end])) for start, end in predicted_events
    )
    rising = np.flatnonzero(predicted & ~np.r_[False, predicted[:-1]])
    false_activations = int(np.sum(~reference[rising]))
    return {
        "samples": int(len(reference)),
        "true_positive_samples": tp,
        "false_positive_samples": fp,
        "false_negative_samples": fn,
        "true_negative_samples": tn,
        "precision": precision,
        "recall": recall,
        "specificity": specificity,
        "balanced_accuracy": (
            (recall + specificity) / 2.0
            if recall is not None and specificity is not None
            else None
        ),
        "f1": f1,
        "accuracy": safe_ratio(tp + tn, len(reference)),
        "reference_contact_fraction": float(np.mean(reference)),
        "predicted_contact_fraction": float(np.mean(predicted)),
        "reference_events": len(reference_events),
        "predicted_events": len(predicted_events),
        "matched_reference_events": matched_reference,
        "matched_predicted_events": matched_predicted,
        "event_recall": safe_ratio(matched_reference, len(reference_events)),
        "event_precision": safe_ratio(matched_predicted, len(predicted_events)),
        "false_contact_activations": false_activations,
        "onset_latency_ms": onset_ms,
        "onset_latency_p50_ms": percentile_or_none(onset_ms, 50.0),
        "onset_latency_p95_ms": percentile_or_none(onset_ms, 95.0),
        "onset_latency_max_ms": max(onset_ms, default=None),
        "release_latency_ms": release_ms,
        "release_latency_p50_ms": percentile_or_none(release_ms, 50.0),
        "release_latency_p95_ms": percentile_or_none(release_ms, 95.0),
        "release_latency_max_ms": max(release_ms, default=None),
    }


def merge_contact_metrics(rows):
    counts = {
        key: sum(row[key] for row in rows)
        for key in (
            "samples",
            "true_positive_samples",
            "false_positive_samples",
            "false_negative_samples",
            "true_negative_samples",
            "reference_events",
            "predicted_events",
            "matched_reference_events",
            "matched_predicted_events",
            "false_contact_activations",
        )
    }
    tp = counts["true_positive_samples"]
    fp = counts["false_positive_samples"]
    fn = counts["false_negative_samples"]
    tn = counts["true_negative_samples"]
    precision = safe_ratio(tp, tp + fp)
    recall = safe_ratio(tp, tp + fn)
    specificity = safe_ratio(tn, tn + fp)
    onset = [value for row in rows for value in row["onset_latency_ms"]]
    release = [value for row in rows for value in row["release_latency_ms"]]
    counts.update(
        {
            "precision": precision,
            "recall": recall,
            "specificity": specificity,
            "balanced_accuracy": (
                (recall + specificity) / 2.0
                if recall is not None and specificity is not None
                else None
            ),
            "f1": (
                2.0 * precision * recall / (precision + recall)
                if precision is not None and recall is not None and precision + recall
                else None
            ),
            "accuracy": safe_ratio(tp + tn, tp + fp + fn + tn),
            "reference_contact_fraction": safe_ratio(tp + fn, tp + fp + fn + tn),
            "predicted_contact_fraction": safe_ratio(tp + fp, tp + fp + fn + tn),
            "event_recall": safe_ratio(
                counts["matched_reference_events"], counts["reference_events"]
            ),
            "event_precision": safe_ratio(
                counts["matched_predicted_events"], counts["predicted_events"]
            ),
            "onset_latency_ms": onset,
            "onset_latency_p50_ms": percentile_or_none(onset, 50.0),
            "onset_latency_p95_ms": percentile_or_none(onset, 95.0),
            "onset_latency_max_ms": max(onset, default=None),
            "release_latency_ms": release,
            "release_latency_p50_ms": percentile_or_none(release, 50.0),
            "release_latency_p95_ms": percentile_or_none(release, 95.0),
            "release_latency_max_ms": max(release, default=None),
        }
    )
    return counts


class HighRateLogs:
    def __init__(self, root):
        self.logs = []
        for path in sorted((root / "observer_logs").glob("*/*.npz")):
            with np.load(path, allow_pickle=False) as data:
                self.logs.append(
                    {
                        "path": str(path),
                        "t": np.asarray(data["source_time_s"], dtype=np.float64),
                        "q": np.asarray(data["q"], dtype=np.float64),
                        "dq": np.asarray(data["dq"], dtype=np.float64),
                    }
                )

    def interpolate(self, times):
        for log in self.logs:
            source = log["t"]
            if source[0] <= times[0] and source[-1] >= times[-1]:
                indices = nearest_indices(source, times)
                q = np.column_stack(
                    [np.interp(times, source, log["q"][:, axis]) for axis in range(6)]
                )
                dq = np.column_stack(
                    [np.interp(times, source, log["dq"][:, axis]) for axis in range(6)]
                )
                gap_ms = np.abs(source[indices] - times) * 1000.0
                return q, dq, log["path"], gap_ms
        return None


def fallback_joint_features(episode, times):
    joint_t = read_array(episode, "robot/joint_time_stamps.zarr")
    joint_q = np.deg2rad(read_array(episode, "robot/joint_deg.zarr"))
    if joint_q.shape != (len(joint_t), 6) or np.any(np.diff(joint_t) <= 0.0):
        raise RuntimeError(f"invalid joint stream: {episode}")
    joint_dq = np.gradient(joint_q, joint_t, axis=0, edge_order=2)
    q = np.column_stack(
        [np.interp(times, joint_t, joint_q[:, axis]) for axis in range(6)]
    )
    dq = np.column_stack(
        [np.interp(times, joint_t, joint_dq[:, axis]) for axis in range(6)]
    )
    gap_ms = np.abs(joint_t[nearest_indices(joint_t, times)] - times) * 1000.0
    return q, dq, str(episode / "robot/joint_deg.zarr"), gap_ms


def causal_features(q, dq, times):
    dt = np.diff(times)
    qdd = np.zeros_like(dq)
    valid = (dt > 0.0) & (dt <= 0.05)
    qdd[1:][valid] = (dq[1:][valid] - dq[:-1][valid]) / dt[valid, None]
    return np.column_stack((q, dq, qdd)).astype(np.float32)


def predict_episode(predictor, q, dq, times):
    history = predictor.history
    features = causal_features(q, dq, times)
    windows = np.lib.stride_tricks.sliding_window_view(
        features, history, axis=0
    ).transpose(0, 2, 1)
    projected = project_feature_windows(windows, predictor.mode)
    learned = []
    with torch.inference_mode():
        for start in range(0, len(projected), 8192):
            learned.append(
                predictor.model(torch.from_numpy(projected[start : start + 8192]))
                .detach()
                .cpu()
                .numpy()
            )
    prediction = np.full((len(times), 6), np.nan, dtype=np.float64)
    prediction[history - 1 :] = np.concatenate(learned).astype(np.float64)
    if predictor.gravity_model is not None:
        prediction[history - 1 :] += np.stack(
            [predictor.gravity_model.predict(value) for value in q[history - 1 :]]
        )
    return prediction


def replay_detector(force_norm, times):
    detector = SchmittContactDetector(
        FORCE_ON_N, FORCE_OFF_N, CONTACT_HOLD_S, FREE_HOLD_S
    )
    state = np.zeros(len(times), dtype=bool)
    for index, (value, stamp) in enumerate(zip(force_norm, times)):
        if np.isfinite(value):
            state[index] = detector.update(value, stamp)
    return state


def metric_from_error(error):
    metrics = error_metrics(error, np.zeros_like(error))
    force_norm = np.linalg.norm(error[:, :3], axis=1)
    metrics.update(
        {
            "force_norm_mean_n": float(np.mean(force_norm)),
            "force_within_1n_fraction": float(np.mean(force_norm <= 1.0)),
            "force_within_2n_fraction": float(np.mean(force_norm <= 2.0)),
            "force_within_3n_fraction": float(np.mean(force_norm <= 3.0)),
            "force_within_4n_fraction": float(np.mean(force_norm <= 4.0)),
        }
    )
    return metrics


def diagnostic_gate(overall, episodes):
    failed = [
        row["episode"]
        for row in episodes
        if row["free_space_metrics"]["force_norm_p95_n"]
        > FORCE_GROUP_P95_LIMIT_N
    ]
    failures = []
    if overall["force_norm_p99_n"] > FORCE_P99_LIMIT_N:
        failures.append("aggregate force p99 exceeds 1 N")
    if overall["force_norm_max_n"] > FORCE_HARD_MAX_LIMIT_N:
        failures.append("aggregate force hard max exceeds 2 N")
    if failed:
        failures.append("one or more episode FREE p95 values exceed 1 N")
    return {
        "passed": not failures,
        "diagnostic_only": True,
        "limits": {
            "aggregate_force_p99_n": FORCE_P99_LIMIT_N,
            "episode_free_force_p95_n": FORCE_GROUP_P95_LIMIT_N,
            "hard_max_n": FORCE_HARD_MAX_LIMIT_N,
        },
        "failed_episode_count": len(failed),
        "failed_episodes": failed,
        "failures": failures,
    }


def shade_contact(axis, times, reference):
    for start, end in binary_intervals(reference):
        right = times[end - 1] if end == len(times) else times[end]
        axis.axvspan(times[start], right, color="tab:red", alpha=0.08, linewidth=0)


def plot_free_episode(pdf, row, values):
    time = values["time"]
    raw = values["raw"]
    prediction = values["prediction"]
    reference = values["reference"]
    score = values["score"]
    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    colors = ("tab:red", "tab:green", "tab:blue")
    for index, color in enumerate(colors):
        axes[0].plot(time, raw[:, index], color=color, linewidth=0.7, label=f"{AXES[index]} measured")
        axes[0].plot(time, prediction[:, index], color=color, linestyle="--", linewidth=0.9, label=f"{AXES[index]} predicted")
        axes[1].plot(time, raw[:, index + 3], color=color, linewidth=0.7, label=f"{AXES[index + 3]} measured")
        axes[1].plot(time, prediction[:, index + 3], color=color, linestyle="--", linewidth=0.9, label=f"{AXES[index + 3]} predicted")
    axes[2].plot(time, score, color="black", linewidth=0.8, label="|measured force - prediction|")
    axes[2].axhline(1.0, color="tab:orange", linestyle="--", linewidth=1.0, label="1 N")
    axes[2].axhline(2.0, color="tab:red", linestyle="--", linewidth=1.0, label="2 N")
    for axis in axes:
        shade_contact(axis, time, reference)
        axis.grid(alpha=0.2)
        axis.legend(loc="upper right", ncol=3, fontsize=7)
    axes[0].set_ylabel("Force [N]")
    axes[1].set_ylabel("Moment [Nm]")
    axes[2].set_ylabel("Residual force [N]")
    axes[2].set_xlabel("Episode time [s]")
    metrics = row["free_space_metrics"]
    figure.suptitle(
        f"{row['episode']} — current free-space model replay\n"
        f"{FIDELITY_LABELS[row['input_fidelity']]}; reference-FREE "
        f"RMSE/p95/p99/max={metrics['force_norm_rmse_n']:.3f}/"
        f"{metrics['force_norm_p95_n']:.3f}/{metrics['force_norm_p99_n']:.3f}/"
        f"{metrics['force_norm_max_n']:.3f} N; red shade=stored reference CONTACT",
        fontsize=10,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    pdf.savefig(figure)
    plt.close(figure)


def plot_contact_episode(pdf, row, values):
    time = values["time"]
    score = values["score"]
    reference = values["reference"]
    predicted = values["predicted_contact"]
    figure, axes = plt.subplots(2, 1, figsize=(12, 6.8), sharex=True)
    axes[0].plot(time, score, color="black", linewidth=0.8, label="current residual force norm")
    axes[0].axhline(FORCE_ON_N, color="tab:red", linestyle="--", label="ON 2.0 N")
    axes[0].axhline(FORCE_OFF_N, color="tab:blue", linestyle="--", label="OFF 1.2 N")
    shade_contact(axes[0], time, reference)
    axes[0].set_ylabel("Contact score [N]")
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[0].grid(alpha=0.2)
    axes[1].step(time, reference.astype(float), where="post", color="tab:blue", linewidth=1.0, label="stored reference state")
    axes[1].step(time, predicted.astype(float) * 0.8, where="post", color="tab:orange", linewidth=1.0, label="current replay state (0.8=CONTACT)")
    axes[1].fill_between(time, 0.0, 1.0, where=reference != predicted, color="tab:red", alpha=0.18, step="post", label="disagreement")
    axes[1].set_yticks((0.0, 0.8, 1.0), ("FREE", "current CONTACT", "reference CONTACT"))
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_xlabel("Episode time [s]")
    axes[1].grid(alpha=0.2)
    axes[1].legend(loc="upper right", fontsize=8)
    metrics = row["contact_metrics"]
    figure.suptitle(
        f"{row['episode']} — current contact observer replay vs stored pseudo-label\n"
        f"precision={metrics['precision'] or 0.0:.3f}, recall={metrics['recall'] or 0.0:.3f}, "
        f"F1={metrics['f1'] or 0.0:.3f}, false activations={metrics['false_contact_activations']}",
        fontsize=10,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    pdf.savefig(figure)
    plt.close(figure)


def plot_free_summary(path, episodes, errors_by_fidelity):
    figure, axes = plt.subplots(3, 1, figsize=(13, 11))
    x = np.arange(len(episodes))
    exact = np.asarray([row["input_fidelity"] == "high_rate_observer_log" for row in episodes])
    for name, key, color in (
        ("p95", "force_norm_p95_n", "tab:blue"),
        ("p99", "force_norm_p99_n", "tab:orange"),
        ("max", "force_norm_max_n", "tab:red"),
    ):
        values = [row["free_space_metrics"][key] for row in episodes]
        axes[0].plot(x, values, label=name, color=color, linewidth=0.9)
    axes[0].scatter(x[~exact], np.zeros(np.sum(~exact)), marker="x", s=12, color="black", label="30 Hz input")
    axes[0].axhline(1.0, color="tab:orange", linestyle="--", linewidth=1.0)
    axes[0].axhline(2.0, color="tab:red", linestyle="--", linewidth=1.0)
    axes[0].set_ylabel("Reference-FREE error [N]")
    axes[0].set_xlabel("Episode index")
    axes[0].legend(ncol=5, fontsize=8)
    axes[0].grid(alpha=0.2)
    for fidelity, errors in errors_by_fidelity.items():
        norm = np.linalg.norm(np.concatenate(errors)[:, :3], axis=1)
        ordered = np.sort(norm)
        axes[1].plot(ordered, np.arange(1, len(ordered) + 1) / len(ordered), label=FIDELITY_LABELS[fidelity])
    axes[1].axvline(1.0, color="tab:orange", linestyle="--")
    axes[1].axvline(2.0, color="tab:red", linestyle="--")
    axes[1].set_xlim(left=0.0)
    axes[1].set_xlabel("Reference-FREE force error [N]")
    axes[1].set_ylabel("Empirical CDF")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.2)
    all_error = np.concatenate([value for values in errors_by_fidelity.values() for value in values])
    axis_rmse = np.sqrt(np.mean(np.square(all_error), axis=0))
    axes[2].bar(AXES, axis_rmse, color=("tab:blue",) * 3 + ("tab:green",) * 3)
    axes[2].set_ylabel("Axis RMSE [N or Nm]")
    axes[2].grid(axis="y", alpha=0.2)
    figure.suptitle("Logistic-box IL episodes — current free-space wrench model summary")
    figure.tight_layout(rect=(0, 0, 1, 0.97))
    figure.savefig(path, dpi=150)
    plt.close(figure)


def plot_stable_free_summary(path, episodes, aggregate_by_guard, errors_by_fidelity):
    figure, axes = plt.subplots(3, 1, figsize=(13, 11))
    guards = np.asarray(FREE_GUARD_MS)
    for label, key, color in (
        ("RMSE", "force_norm_rmse_n", "tab:blue"),
        ("p95", "force_norm_p95_n", "tab:orange"),
        ("p99", "force_norm_p99_n", "tab:red"),
    ):
        axes[0].plot(
            guards,
            [aggregate_by_guard[str(value)][key] for value in guards],
            marker="o",
            label=label,
            color=color,
        )
    axes[0].axvline(PRIMARY_FREE_GUARD_MS, color="black", linestyle="--")
    axes[0].set_xlabel("Distance from stored CONTACT [ms]")
    axes[0].set_ylabel("Force error [N]")
    axes[0].legend(ncol=3)
    axes[0].grid(alpha=0.2)

    x = np.arange(len(episodes))
    p95 = [
        row["free_space_metrics_by_guard_ms"][str(PRIMARY_FREE_GUARD_MS)][
            "force_norm_p95_n"
        ]
        for row in episodes
    ]
    axes[1].plot(x, p95, color="tab:blue", linewidth=0.9)
    axes[1].axhline(1.0, color="tab:orange", linestyle="--", label="1 N")
    axes[1].set_xlabel("Episode index")
    axes[1].set_ylabel(f"p95 at {PRIMARY_FREE_GUARD_MS} ms guard [N]")
    axes[1].legend()
    axes[1].grid(alpha=0.2)

    for fidelity, errors in errors_by_fidelity.items():
        norm = np.linalg.norm(np.concatenate(errors)[:, :3], axis=1)
        ordered = np.sort(norm)
        axes[2].plot(
            ordered,
            np.arange(1, len(ordered) + 1) / len(ordered),
            label=FIDELITY_LABELS[fidelity],
        )
    axes[2].axvline(1.0, color="tab:orange", linestyle="--")
    axes[2].axvline(2.0, color="tab:red", linestyle="--")
    axes[2].set_xlim(left=0.0)
    axes[2].set_xlabel(f"Force error at {PRIMARY_FREE_GUARD_MS} ms guard [N]")
    axes[2].set_ylabel("Empirical CDF")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.2)
    figure.suptitle("Stable non-contact free-space wrench accuracy")
    figure.tight_layout(rect=(0, 0, 1, 0.97))
    figure.savefig(path, dpi=150)
    plt.close(figure)


def plot_contact_summary(path, episodes, aggregate):
    figure, axes = plt.subplots(3, 1, figsize=(13, 11))
    x = np.arange(len(episodes))
    for label, key, color in (
        ("precision", "precision", "tab:blue"),
        ("recall", "recall", "tab:orange"),
        ("F1", "f1", "tab:green"),
    ):
        values = [row["contact_metrics"][key] or 0.0 for row in episodes]
        axes[0].plot(x, values, label=label, color=color, linewidth=0.9)
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_xlabel("Episode index")
    axes[0].set_ylabel("State agreement metric")
    axes[0].legend(ncol=3)
    axes[0].grid(alpha=0.2)
    matrix = np.asarray(
        [
            [aggregate["true_negative_samples"], aggregate["false_positive_samples"]],
            [aggregate["false_negative_samples"], aggregate["true_positive_samples"]],
        ]
    )
    image = axes[1].imshow(matrix, cmap="Blues")
    for row in range(2):
        for column in range(2):
            axes[1].text(column, row, f"{matrix[row, column]:,}", ha="center", va="center")
    axes[1].set_xticks((0, 1), ("Pred FREE", "Pred CONTACT"))
    axes[1].set_yticks((0, 1), ("Ref FREE", "Ref CONTACT"))
    axes[1].set_title("Sample confusion matrix")
    figure.colorbar(image, ax=axes[1], fraction=0.025)
    onset = aggregate["onset_latency_ms"]
    release = aggregate["release_latency_ms"]
    if onset:
        axes[2].hist(onset, bins=40, alpha=0.65, label="onset")
    if release:
        axes[2].hist(release, bins=40, alpha=0.65, label="release")
    axes[2].set_xlabel("Matched event latency [ms]")
    axes[2].set_ylabel("Events")
    axes[2].legend()
    axes[2].grid(alpha=0.2)
    figure.suptitle("Current contact observer replay vs stored contact-state pseudo-label")
    figure.tight_layout(rect=(0, 0, 1, 0.97))
    figure.savefig(path, dpi=150)
    plt.close(figure)


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, float):
        return f"{value:.9g}"
    return value


def write_csv(path, rows, fields):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[name for name, _ in fields])
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    name: csv_value(getter(row))
                    for name, getter in fields
                }
            )


def fmt(value, digits=3):
    return "N/A" if value is None else f"{value:.{digits}f}"


def free_readme(report):
    stable = report["aggregate_by_contact_guard_ms"][str(PRIMARY_FREE_GUARD_MS)]
    gate = report["stable_free_diagnostic_gate"]
    by = report["by_input_fidelity_and_contact_guard_ms"][
        str(PRIMARY_FREE_GUARD_MS)
    ]
    rows = []
    for key in ("all", "high_rate_observer_log", "joint_30hz_interpolated"):
        value = stable if key == "all" else by[key]
        label = "전체" if key == "all" else FIDELITY_LABELS[key]
        rows.append(
            f"| {label} | {value['samples']:,} | "
            f"{value['force_norm_mean_n']:.3f} | "
            f"{value['force_within_1n_fraction']:.1%} | "
            f"{value['force_within_2n_fraction']:.1%} | "
            f"{value['force_within_3n_fraction']:.1%} | "
            f"{value['force_within_4n_fraction']:.1%} | "
            f"{value['force_norm_max_n']:.3f} |"
        )
    guard_rows = []
    for guard_ms in FREE_GUARD_MS:
        value = report["aggregate_by_contact_guard_ms"][str(guard_ms)]
        guard_rows.append(
            f"| {guard_ms} | {value['samples']:,} | "
            f"{value['force_norm_mean_n']:.3f} | "
            f"{value['force_within_1n_fraction']:.1%} | "
            f"{value['force_within_2n_fraction']:.1%} | "
            f"{value['force_within_3n_fraction']:.1%} | "
            f"{value['force_within_4n_fraction']:.1%} | "
            f"{value['force_norm_max_n']:.3f} |"
        )
    return f"""# 모방학습 episode의 free-space wrench 모델 검증

## 결론

현재 diagnostic 모델을 102개 logistic-box 모방학습 episode에 offline replay했다.
주 지표인 **저장 CONTACT에서 200 ms 이상 떨어진 안정 FREE 구간**의 aggregate force
RMSE/p95/p99/max는 `{stable['force_norm_rmse_n']:.3f}/`
`{stable['force_norm_p95_n']:.3f}/{stable['force_norm_p99_n']:.3f}/`
`{stable['force_norm_max_n']:.3f} N`이다.
현재 기준을 그대로 대입한 진단 판정은 **{'PASS' if gate['passed'] else 'FAIL'}**이며,
episode p95 실패는 `{gate['failed_episode_count']}/102`개다. 이 결과는 contact가 없는
독립 zero-set test가 아니므로 정식 `FS-03` 승격 evidence는 아니다.

| 200 ms 안정 FREE 입력 범위 | samples | 평균 오차 [N] | <=1 N | <=2 N | <=3 N | <=4 N | 최대 [N] |
|---|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

전환 주변 제외 폭에 따른 민감도는 다음과 같다. 0 ms는 저장 state가 FREE인 모든 구간이다.

| CONTACT guard [ms] | samples | 평균 오차 [N] | <=1 N | <=2 N | <=3 N | <=4 N | 최대 [N] |
|---:|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(guard_rows)}

현재 gate: aggregate p99 `<=1 N`, episode FREE p95 `<=1 N`, hard max `<=2 N`.

## 산출물

- [전체 요약 plot](summary.png)
- [안정 비접촉 분석 plot](stable_free_summary.png)
- [102개 episode별 plot PDF](episode_plots.pdf): PDF page `n+1`이 `episode_nnn`
- [episode별 수치 CSV](episode_metrics.csv)
- [기계 판독용 전체 report](analysis.json)

각 episode plot은 sensor-frame 6축 measured/predicted wrench와 force residual을 표시한다.
붉은 음영은 저장된 기존 observer의 CONTACT 구간이며, 정확도 지표에서는 제외했다.

## 입력 재구성과 제한

- 모든 episode에 `free_space_wrench_prediction.zarr`가 없어 모델 SHA-256
  `{report['model']['sha256']}`의 출력을 다시 계산했다.
- `{report['input_fidelity_counts']['high_rate_observer_log']}`개는 별도 262.5 Hz observer
  log의 실제 `q,dq`를 사용했다.
- `{report['input_fidelity_counts']['joint_30hz_interpolated']}`개는 high-rate log가 없어
  저장된 30 Hz joint를 보간하고 `dq,qdd`를 재구성했다. 이 subset은 입력 근사 오차가
  포함되므로 별도 집계했다.
- 동일 source identity의 valid→invalid 상태 전이 `{report['deduplicated_contact_samples']}`개는
  runtime의 duplicate-source 계약과 같이 첫 valid row만 사용했다.
- FREE/CONTACT 구분은 모방학습 data에 저장된 이전 observer state를 사용했다. 이는
  독립 ground truth가 아니다. 전환 오염을 줄이기 위해 200 ms guard를 주 지표로 삼았지만,
  잘못 저장된 장시간 FREE/CONTACT 구간 자체는 교정할 수 없다.
- model bundle은 `approved=false`인 diagnostic artifact이며 runtime 설정은 변경하지 않았다.

## 육하원칙

- 누가/언제: 이 offline 분석기가 {report['created_utc']}에 기존 저장 data만 읽었다.
- 어디서/무엇을: `{report['dataset']}`의 102개 episode에 현재 free-space model을 replay했다.
- 어떻게: episode 시작 전 pre-roll로 32-sample history를 채우고, raw FT에서 예측 wrench를
  뺀 force norm을 reference-FREE 구간에서 집계했다.
- 왜: 모방학습 domain에서 모델의 zero-set·자세·동작 일반화 성능을 확인하기 위해서다.
"""


def contact_readme(report):
    overall = report["aggregate"]
    by = report["by_input_fidelity"]
    audit = report["ground_truth_audit"]
    rows = []
    for key in ("all", "high_rate_observer_log", "joint_30hz_interpolated"):
        value = overall if key == "all" else by[key]
        label = "전체" if key == "all" else FIDELITY_LABELS[key]
        rows.append(
            f"| {label} | {value['samples']:,} | {fmt(value['precision'])} | "
            f"{fmt(value['recall'])} | {fmt(value['f1'])} | "
            f"{fmt(value['accuracy'])} | {fmt(value['balanced_accuracy'])} | "
            f"{value['false_contact_activations']:,} |"
        )
    return f"""# 모방학습 episode의 contact observer 검증

## 결론

**실제 물리 접촉 정답과의 정확도: 계산 불가(NOT EVALUABLE).**
102개 episode를 확인했지만 독립적인 same-clock contact interval, 수동 annotation 또는
외부 접촉 센서 label이 있는 episode는
`{audit['episodes_with_independent_ground_truth']}/{audit['episodes_checked']}`개다.
반면 `{audit['episodes_using_observer_output_as_reference']}`개 모두 저장 state의 source가
`/contact_observer/right/observation`이므로 이를 실제 정답이라고 간주하면 순환 검증이 된다.

아래 수치는 실제 정답 정확도가 아니라 기존 IL observer 출력과의 **참고용 일치도**다.
현재 free-space 모델 residual에 현행 Schmitt detector(`ON 2.0 N / OFF 1.2 N`,
`8/20 ms` hold)를 적용하고, 모방학습 data에 저장된 기존 contact state와 비교했다.
전체 sample precision/recall/F1/accuracy는 `{fmt(overall['precision'])}/`
`{fmt(overall['recall'])}/{fmt(overall['f1'])}/{fmt(overall['accuracy'])}`이며
balanced accuracy는 `{fmt(overall['balanced_accuracy'])}`, false CONTACT activation은
`{overall['false_contact_activations']}`회다.

**이 수치는 물리 contact 정확도 판정이 아니라 IL 입력 contact channel과의 일치도다.**
저장 state도 이전 free-space model이 만든 값이고 독립 접촉 label이나 같은 시계의 수동
contact interval이 없다. 따라서 precision/recall이 높더라도 정식 `CO-04` PASS로
판정할 수 없다.

| 입력 범위 | samples | precision | recall | F1 | accuracy | balanced accuracy | false activations |
|---|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

Reference event `{overall['reference_events']}`개 중 current replay와 겹친 event는
`{overall['matched_reference_events']}`개(event recall `{fmt(overall['event_recall'])}`)다.
일치 event의 onset latency p50/p95/max는
`{fmt(overall['onset_latency_p50_ms'])}/{fmt(overall['onset_latency_p95_ms'])}/`
`{fmt(overall['onset_latency_max_ms'])} ms`다.

## 산출물

- [전체 요약 plot](summary.png)
- [102개 episode별 plot PDF](episode_plots.pdf): PDF page `n+1`이 `episode_nnn`
- [episode별 수치 CSV](episode_metrics.csv)
- [기계 판독용 전체 report](analysis.json)

각 episode plot은 current residual force norm, ON/OFF threshold, 저장 reference state와
current replay state를 함께 표시한다. 붉은 영역은 두 state의 불일치 구간이다.

## 입력과 제한

- 모델 SHA-256: `{report['model']['sha256']}` (`approved=false` diagnostic bundle)
- 262.5 Hz `q,dq` 직접 replay: `{report['input_fidelity_counts']['high_rate_observer_log']}`개
- 30 Hz joint 보간 replay: `{report['input_fidelity_counts']['joint_30hz_interpolated']}`개
- episode별 detector는 저장된 약 1초 pre-roll 시작에서 reset했다. 정식 runtime의
  episode 사이 연속 state와는 다르며 독립 episode 비교를 위한 계약이다.
- 동일 source identity 중복 `{report['deduplicated_contact_samples']}`개는 첫 valid row만
  사용했다.
- 물리 정확도 확정에는 동일 시계의 독립 contact interval annotation이 필요하다.

## 육하원칙

- 누가/언제: 이 offline 분석기가 {report['created_utc']}에 저장 data를 replay했다.
- 어디서/무엇을: `{report['dataset']}`의 102개 episode에서 current contact state를 계산했다.
- 어떻게: `W_contact = W_raw - W_free_hat`의 force norm에 현행 Schmitt detector를 적용하고
  저장된 IL contact channel과 sample/event 단위로 비교했다.
- 왜: 새 model/observer가 기존 모방학습 observation과 얼마나 호환되는지 확인하기 위해서다.
"""


def self_check():
    reference = np.asarray([0, 0, 1, 1, 0, 0], dtype=bool)
    predicted = np.asarray([0, 1, 1, 1, 0, 0], dtype=bool)
    times = np.arange(6, dtype=np.float64) * 0.01
    metrics = contact_metrics(reference, predicted, times)
    assert metrics["true_positive_samples"] == 2
    assert metrics["false_positive_samples"] == 1
    assert metrics["balanced_accuracy"] == 0.875
    assert metrics["reference_events"] == 1
    assert metrics["matched_reference_events"] == 1
    assert guarded_free_mask(reference, times, 10).tolist() == [
        True,
        False,
        False,
        False,
        False,
        True,
    ]
    simple_error = np.zeros((2, 6), dtype=np.float64)
    simple_error[:, 0] = (0.5, 2.5)
    simple_metrics = metric_from_error(simple_error)
    assert simple_metrics["force_norm_mean_n"] == 1.5
    assert simple_metrics["force_within_1n_fraction"] == 0.5
    assert simple_metrics["force_within_3n_fraction"] == 1.0
    assert nearest_indices(np.asarray([0.0, 1.0, 2.0]), np.asarray([0.2, 1.8])).tolist() == [0, 2]
    q = np.zeros((3, 6))
    dq = np.asarray([[0.0] * 6, [1.0] * 6, [3.0] * 6])
    features = causal_features(q, dq, np.asarray([1.0, 1.01, 1.02]))
    assert np.allclose(features[1, 12:], 100.0)
    assert np.allclose(features[2, 12:], 200.0)
    features = np.zeros((40, 18), dtype=np.float32)
    windows = np.lib.stride_tricks.sliding_window_view(
        features, 32, axis=0
    ).transpose(0, 2, 1)
    assert windows.shape == (9, 32, 18)
    assert project_feature_windows(windows, "short_multiscale").shape == (9, 54)
    print("logistic-box IL replay self-check: PASS")


def require_empty_output(path):
    if path.exists() and (not path.is_dir() or any(path.iterdir())):
        raise FileExistsError(f"refusing to overwrite non-empty output: {path}")


def install_output(source, target):
    target.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        path.rename(target / path.name)


def analyze(args):
    data_root = args.data.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    free_target = args.free_output.expanduser().resolve()
    contact_target = args.contact_output.expanduser().resolve()
    if not data_root.is_dir() or not model_path.is_file():
        raise RuntimeError("data directory and model.ts are required")
    require_empty_output(free_target)
    require_empty_output(contact_target)
    if free_target.parent != contact_target.parent:
        raise RuntimeError("validation output directories must share one parent")

    episodes = sorted(data_root.glob("episode_[0-9][0-9][0-9]"))
    if [path.name for path in episodes] != [f"episode_{index:03d}" for index in range(102)]:
        raise RuntimeError("expected contiguous episode_000 through episode_101")
    predictor = BundlePredictor(model_path, require_approved=False)
    logs = HighRateLogs(data_root)
    created_utc = datetime.now(timezone.utc).isoformat()
    script_hash = file_sha256(Path(__file__))
    episode_rows = []
    errors_by_fidelity = {key: [] for key in FIDELITY_LABELS}
    errors_by_guard = {
        str(guard_ms): {key: [] for key in FIDELITY_LABELS}
        for guard_ms in FREE_GUARD_MS
    }
    fidelity_counts = {key: 0 for key in FIDELITY_LABELS}

    free_target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".il_replay_", dir=free_target.parent) as temporary:
        temporary = Path(temporary)
        free_output = temporary / free_target.name
        contact_output = temporary / contact_target.name
        free_output.mkdir()
        contact_output.mkdir()
        with PdfPages(free_output / "episode_plots.pdf") as free_pdf, PdfPages(
            contact_output / "episode_plots.pdf"
        ) as contact_pdf:
            for number, episode in enumerate(episodes):
                meta = read_json(episode / "meta.json")
                source_t = read_array(episode, "contact/source_time_stamps.zarr")
                receive_t = read_array(episode, "contact/receive_time_stamps.zarr")
                source_sequence = read_array(
                    episode, "contact/source_sequences.zarr"
                ).astype(np.int64)
                reference = read_array(episode, "contact/contact_state.zarr").astype(bool)
                stored_valid = read_array(episode, "contact/contact_valid.zarr").astype(bool)
                stored_ready = read_array(episode, "contact/contact_model_ready.zarr").astype(bool)
                if not all(
                    len(value) == len(source_t)
                    for value in (
                        receive_t,
                        source_sequence,
                        reference,
                        stored_valid,
                        stored_ready,
                    )
                ):
                    raise RuntimeError(f"contact array length mismatch: {episode}")
                same_identity = (
                    (np.diff(source_t) == 0.0)
                    & (np.diff(source_sequence) == 0)
                )
                if np.any(
                    (np.diff(source_t) <= 0.0) & ~same_identity
                ) or np.any((np.diff(source_sequence) <= 0) & ~same_identity):
                    raise RuntimeError(f"contact source identity moved backwards: {episode}")
                keep = np.r_[True, ~same_identity]
                deduplicated_samples = int(np.sum(~keep))
                source_t = source_t[keep]
                receive_t = receive_t[keep]
                reference = reference[keep]
                stored_valid = stored_valid[keep]
                stored_ready = stored_ready[keep]
                if np.any(np.diff(receive_t) <= 0.0):
                    raise RuntimeError(f"contact timestamps do not increase: {episode}")

                high_rate = logs.interpolate(source_t)
                if high_rate is None:
                    q, dq, feature_source, feature_gap_ms = fallback_joint_features(
                        episode, source_t
                    )
                    fidelity = "joint_30hz_interpolated"
                else:
                    q, dq, feature_source, feature_gap_ms = high_rate
                    fidelity = "high_rate_observer_log"
                fidelity_counts[fidelity] += 1

                ft_t = read_array(episode, "ft/wrench_time_stamps.zarr")
                raw_stream = read_array(episode, "ft/wrench_raw.zarr")
                if raw_stream.shape != (len(ft_t), 6):
                    raise RuntimeError(f"invalid raw FT stream: {episode}")
                ft_indices = nearest_indices(ft_t, receive_t)
                raw = raw_stream[ft_indices]
                ft_gap_ms = np.abs(ft_t[ft_indices] - receive_t) * 1000.0
                prediction = predict_episode(predictor, q, dq, source_t)
                residual = raw - prediction
                force_norm = np.linalg.norm(residual[:, :3], axis=1)
                predicted_contact = replay_detector(force_norm, source_t)

                official = (
                    (source_t >= float(meta["created_at"]))
                    & (source_t <= float(meta["stopped_at"]))
                )
                usable = official & stored_valid & stored_ready & np.isfinite(prediction[:, 0])
                free = usable & ~reference
                if np.sum(free) < 2 or np.sum(usable & reference) < 2:
                    raise RuntimeError(f"episode lacks usable FREE or CONTACT samples: {episode}")
                free_error = residual[free]
                free_metrics = metric_from_error(free_error)
                guarded_masks = {
                    str(guard_ms): usable
                    & guarded_free_mask(reference, source_t, guard_ms)
                    for guard_ms in FREE_GUARD_MS
                }
                if any(np.sum(mask) < 2 for mask in guarded_masks.values()):
                    raise RuntimeError(f"episode lacks stable FREE samples: {episode}")
                free_metrics_by_guard = {
                    key: metric_from_error(residual[mask])
                    for key, mask in guarded_masks.items()
                }
                episode_gate = (
                    free_metrics["force_norm_p95_n"] <= FORCE_GROUP_P95_LIMIT_N
                    and free_metrics["force_norm_p99_n"] <= FORCE_P99_LIMIT_N
                    and free_metrics["force_norm_max_n"] <= FORCE_HARD_MAX_LIMIT_N
                )
                state_metrics = contact_metrics(
                    reference[usable], predicted_contact[usable], source_t[usable]
                )
                contact_entries = [
                    path.name.lower() for path in (episode / "contact").iterdir()
                ]
                independent_ground_truth_available = any(
                    token in name
                    for name in contact_entries
                    for token in ("ground_truth", "manual", "annotation", "interval")
                )
                row = {
                    "episode": episode.name,
                    "episode_meta_sha256": file_sha256(episode / "meta.json"),
                    "interruption_reason": meta.get("interruption_reason"),
                    "writer_error": meta.get("writer_error"),
                    "input_fidelity": fidelity,
                    "feature_source": feature_source,
                    "feature_alignment_p95_ms": float(np.percentile(feature_gap_ms, 95.0)),
                    "feature_alignment_max_ms": float(np.max(feature_gap_ms)),
                    "ft_alignment_p95_ms": float(np.percentile(ft_gap_ms, 95.0)),
                    "ft_alignment_max_ms": float(np.max(ft_gap_ms)),
                    "official_samples": int(np.sum(official)),
                    "usable_samples": int(np.sum(usable)),
                    "excluded_samples": int(np.sum(official & ~usable)),
                    "deduplicated_samples": deduplicated_samples,
                    "free_samples": int(np.sum(free)),
                    "contact_samples": int(np.sum(usable & reference)),
                    "free_space_metrics": free_metrics,
                    "free_samples_by_guard_ms": {
                        key: int(np.sum(mask)) for key, mask in guarded_masks.items()
                    },
                    "free_space_metrics_by_guard_ms": free_metrics_by_guard,
                    "diagnostic_gate_passed": episode_gate,
                    "contact_metrics": state_metrics,
                    "contact_reference_topic": meta.get("source_topics", {}).get(
                        "contact_observation"
                    ),
                    "independent_ground_truth_available": (
                        independent_ground_truth_available
                    ),
                }
                episode_rows.append(row)
                errors_by_fidelity[fidelity].append(free_error)
                for key, mask in guarded_masks.items():
                    errors_by_guard[key][fidelity].append(residual[mask])

                plot_mask = official & np.isfinite(prediction[:, 0])
                plot_values = {
                    "time": source_t[plot_mask] - float(meta["created_at"]),
                    "raw": raw[plot_mask],
                    "prediction": prediction[plot_mask],
                    "reference": reference[plot_mask],
                    "score": force_norm[plot_mask],
                    "predicted_contact": predicted_contact[plot_mask],
                }
                plot_free_episode(free_pdf, row, plot_values)
                plot_contact_episode(contact_pdf, row, plot_values)
                if number % 10 == 0 or number == len(episodes) - 1:
                    print(f"[replay] {number + 1}/{len(episodes)} {episode.name}", flush=True)

        all_error = np.concatenate(
            [value for values in errors_by_fidelity.values() for value in values]
        )
        aggregate_free = metric_from_error(all_error)
        free_by_fidelity = {
            key: metric_from_error(np.concatenate(values))
            for key, values in errors_by_fidelity.items()
        }
        aggregate_free_by_guard = {
            guard: metric_from_error(
                np.concatenate(
                    [value for values in by_fidelity.values() for value in values]
                )
            )
            for guard, by_fidelity in errors_by_guard.items()
        }
        free_by_fidelity_and_guard = {
            guard: {
                key: metric_from_error(np.concatenate(values))
                for key, values in by_fidelity.items()
            }
            for guard, by_fidelity in errors_by_guard.items()
        }
        primary_guard = str(PRIMARY_FREE_GUARD_MS)
        primary_episode_rows = [
            {
                **row,
                "free_space_metrics": row["free_space_metrics_by_guard_ms"][
                    primary_guard
                ],
            }
            for row in episode_rows
        ]
        free_report = {
            "schema_version": 1,
            "analysis_type": "logistic_box_il_current_free_space_model_replay_v1",
            "created_utc": created_utc,
            "script_sha256": script_hash,
            "dataset": str(data_root),
            "episodes": len(episode_rows),
            "model": {
                "path": str(model_path),
                "sha256": file_sha256(model_path),
                "approved": bool(predictor.metadata.get("approved", False)),
                "ablation": predictor.ablation,
                "history": predictor.history,
                "prediction_contract": predictor.metadata.get("prediction_contract"),
            },
            "evaluation_contract": {
                "target": "ft/wrench_raw.zarr in aft_sensor2 sensor frame",
                "free_reference": "stored legacy observer contact_state == FREE",
                "stable_free_reference": (
                    f"stored FREE at least {PRIMARY_FREE_GUARD_MS} ms from any "
                    "stored CONTACT sample"
                ),
                "contact_guard_sweep_ms": list(FREE_GUARD_MS),
                "score_window": "meta.created_at through meta.stopped_at",
                "pre_roll": "stored pre-roll fills the 32-sample model history",
                "formal_fs03_evidence": False,
            },
            "input_fidelity_counts": fidelity_counts,
            "deduplicated_contact_samples": sum(
                row["deduplicated_samples"] for row in episode_rows
            ),
            "aggregate": aggregate_free,
            "by_input_fidelity": free_by_fidelity,
            "aggregate_by_contact_guard_ms": aggregate_free_by_guard,
            "by_input_fidelity_and_contact_guard_ms": (
                free_by_fidelity_and_guard
            ),
            "diagnostic_gate": diagnostic_gate(aggregate_free, episode_rows),
            "stable_free_diagnostic_gate": diagnostic_gate(
                aggregate_free_by_guard[primary_guard], primary_episode_rows
            ),
            "episodes_detail": episode_rows,
            "limitations": [
                "stored contact_state is a legacy-observer pseudo-label, not independent ground truth",
                "56 episodes reconstruct dq/qdd from 30 Hz joint samples",
                "the current bundle is diagnostic and approved=false",
            ],
        }

        contact_rows = [row["contact_metrics"] for row in episode_rows]
        aggregate_contact = merge_contact_metrics(contact_rows)
        contact_by_fidelity = {
            key: merge_contact_metrics(
                [
                    row["contact_metrics"]
                    for row in episode_rows
                    if row["input_fidelity"] == key
                ]
            )
            for key in FIDELITY_LABELS
        }
        contact_report = {
            "schema_version": 1,
            "analysis_type": "logistic_box_il_current_contact_observer_replay_v1",
            "created_utc": created_utc,
            "script_sha256": script_hash,
            "dataset": str(data_root),
            "episodes": len(episode_rows),
            "model": free_report["model"],
            "detector": {
                "force_on_n": FORCE_ON_N,
                "force_off_n": FORCE_OFF_N,
                "contact_hold_ms": CONTACT_HOLD_S * 1000.0,
                "free_hold_ms": FREE_HOLD_S * 1000.0,
                "reset_contract": "reset at each episode pre-roll start",
            },
            "reference_contract": {
                "source": "contact/contact_state.zarr",
                "kind": "legacy observer pseudo-label consumed by the IL dataset",
                "independent_physical_ground_truth": False,
                "formal_co04_evidence": False,
            },
            "ground_truth_audit": {
                "episodes_checked": len(episode_rows),
                "episodes_with_independent_ground_truth": sum(
                    row["independent_ground_truth_available"]
                    for row in episode_rows
                ),
                "episodes_using_observer_output_as_reference": sum(
                    row["contact_reference_topic"]
                    == "/contact_observer/right/observation"
                    for row in episode_rows
                ),
                "physical_accuracy_computable": False,
                "reason": (
                    "no independent same-clock contact interval, manual annotation, "
                    "or external contact sensor label is stored"
                ),
            },
            "input_fidelity_counts": fidelity_counts,
            "deduplicated_contact_samples": free_report[
                "deduplicated_contact_samples"
            ],
            "aggregate": aggregate_contact,
            "by_input_fidelity": contact_by_fidelity,
            "episodes_detail": episode_rows,
            "limitations": [
                "agreement with stored observer state does not prove physical contact correctness",
                "56 episodes reconstruct dq/qdd from 30 Hz joint samples",
                "episode-wise reset differs from a continuously running observer",
            ],
        }

        plot_free_summary(free_output / "summary.png", episode_rows, errors_by_fidelity)
        plot_stable_free_summary(
            free_output / "stable_free_summary.png",
            episode_rows,
            aggregate_free_by_guard,
            errors_by_guard[primary_guard],
        )
        plot_contact_summary(contact_output / "summary.png", episode_rows, aggregate_contact)
        write_csv(
            free_output / "episode_metrics.csv",
            episode_rows,
            (
                ("episode", lambda row: row["episode"]),
                ("input_fidelity", lambda row: row["input_fidelity"]),
                ("usable_samples", lambda row: row["usable_samples"]),
                ("free_samples", lambda row: row["free_samples"]),
                (
                    "stable_free_samples_200ms",
                    lambda row: row["free_samples_by_guard_ms"]["200"],
                ),
                ("contact_samples", lambda row: row["contact_samples"]),
                ("deduplicated_samples", lambda row: row["deduplicated_samples"]),
                ("force_rmse_n", lambda row: row["free_space_metrics"]["force_norm_rmse_n"]),
                ("force_p95_n", lambda row: row["free_space_metrics"]["force_norm_p95_n"]),
                ("force_p99_n", lambda row: row["free_space_metrics"]["force_norm_p99_n"]),
                ("force_max_n", lambda row: row["free_space_metrics"]["force_norm_max_n"]),
                (
                    "stable_force_mean_200ms_n",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_norm_mean_n"
                    ],
                ),
                (
                    "stable_force_rmse_200ms_n",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_norm_rmse_n"
                    ],
                ),
                (
                    "stable_force_p95_200ms_n",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_norm_p95_n"
                    ],
                ),
                (
                    "stable_force_p99_200ms_n",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_norm_p99_n"
                    ],
                ),
                (
                    "stable_force_max_200ms_n",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_norm_max_n"
                    ],
                ),
                (
                    "stable_force_within_1n_200ms",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_within_1n_fraction"
                    ],
                ),
                (
                    "stable_force_within_2n_200ms",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_within_2n_fraction"
                    ],
                ),
                (
                    "stable_force_within_3n_200ms",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_within_3n_fraction"
                    ],
                ),
                (
                    "stable_force_within_4n_200ms",
                    lambda row: row["free_space_metrics_by_guard_ms"]["200"][
                        "force_within_4n_fraction"
                    ],
                ),
                ("diagnostic_gate", lambda row: row["diagnostic_gate_passed"]),
                ("feature_alignment_p95_ms", lambda row: row["feature_alignment_p95_ms"]),
                ("feature_alignment_max_ms", lambda row: row["feature_alignment_max_ms"]),
                ("ft_alignment_p95_ms", lambda row: row["ft_alignment_p95_ms"]),
                ("ft_alignment_max_ms", lambda row: row["ft_alignment_max_ms"]),
            ),
        )
        write_csv(
            contact_output / "episode_metrics.csv",
            episode_rows,
            (
                ("episode", lambda row: row["episode"]),
                ("input_fidelity", lambda row: row["input_fidelity"]),
                (
                    "independent_ground_truth_available",
                    lambda row: row["independent_ground_truth_available"],
                ),
                ("samples", lambda row: row["contact_metrics"]["samples"]),
                ("deduplicated_samples", lambda row: row["deduplicated_samples"]),
                ("precision", lambda row: row["contact_metrics"]["precision"]),
                ("recall", lambda row: row["contact_metrics"]["recall"]),
                ("specificity", lambda row: row["contact_metrics"]["specificity"]),
                ("f1", lambda row: row["contact_metrics"]["f1"]),
                ("accuracy", lambda row: row["contact_metrics"]["accuracy"]),
                (
                    "balanced_accuracy",
                    lambda row: row["contact_metrics"]["balanced_accuracy"],
                ),
                ("reference_contact_fraction", lambda row: row["contact_metrics"]["reference_contact_fraction"]),
                ("predicted_contact_fraction", lambda row: row["contact_metrics"]["predicted_contact_fraction"]),
                ("false_positive_samples", lambda row: row["contact_metrics"]["false_positive_samples"]),
                ("false_negative_samples", lambda row: row["contact_metrics"]["false_negative_samples"]),
                ("false_contact_activations", lambda row: row["contact_metrics"]["false_contact_activations"]),
                ("reference_events", lambda row: row["contact_metrics"]["reference_events"]),
                ("predicted_events", lambda row: row["contact_metrics"]["predicted_events"]),
                ("event_precision", lambda row: row["contact_metrics"]["event_precision"]),
                ("event_recall", lambda row: row["contact_metrics"]["event_recall"]),
                ("onset_p50_ms", lambda row: row["contact_metrics"]["onset_latency_p50_ms"]),
                ("onset_p95_ms", lambda row: row["contact_metrics"]["onset_latency_p95_ms"]),
                ("onset_max_ms", lambda row: row["contact_metrics"]["onset_latency_max_ms"]),
                ("release_p50_ms", lambda row: row["contact_metrics"]["release_latency_p50_ms"]),
                ("release_p95_ms", lambda row: row["contact_metrics"]["release_latency_p95_ms"]),
                ("release_max_ms", lambda row: row["contact_metrics"]["release_latency_max_ms"]),
            ),
        )
        (free_output / "analysis.json").write_text(
            json.dumps(free_report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        (contact_output / "analysis.json").write_text(
            json.dumps(contact_report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        (free_output / "README.md").write_text(free_readme(free_report), encoding="utf-8")
        (contact_output / "README.md").write_text(
            contact_readme(contact_report), encoding="utf-8"
        )
        install_output(free_output, free_target)
        install_output(contact_output, contact_target)

    print(
        f"[done] free p99={aggregate_free['force_norm_p99_n']:.3f} N, "
        f"contact F1={aggregate_contact['f1'] or 0.0:.3f}, "
        f"outputs={free_target},{contact_target}",
        flush=True,
    )


def main(argv=None):
    args = parse_args(argv)
    if args.self_check:
        self_check()
        return 0
    analyze(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
